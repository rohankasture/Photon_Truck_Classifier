#define ARM_MATH_CM3
#define STAGES 4
#include "arm_math.h"
#include "Particle.h"
#include "adc_hal.h"
#include "gpio_hal.h"
#include "pinmap_hal.h"
#include "pinmap_impl.h"

void buttonHandler(system_event_t event, int data);
bool decisionTree();
const size_t SAMPLE_BUF_SIZE = 4096;
const int SAMPLE_PIN = A5;
const long SAMPLE_RATE = 44100;
const unsigned long MAX_RECORDING_LENGTH_MS = 3000;
IPAddress serverAddr = IPAddress(18,218,51,247);
int serverPort = 16666;
uint16_t samples[SAMPLE_BUF_SIZE];
TCPClient client;
unsigned long recordingStart;
enum State { STATE_WAITING, STATE_CONNECT, STATE_RUNNING, STATE_FINISH };
State state = STATE_WAITING;

//Filter coeffs Cheby1
float32_t filter_coeffs[STAGES*5] = {
    2.15994126E-02/2, 0.00000000E+00, -2.15994126E-02/2, 1.97853149E+00/2, -9.88507380E-01/2,
    2.15994126E-02/2, 0.00000000E+00, -2.15994126E-02/2, 1.99004001E+00/2, -9.93358188E-01/2,
    1.39289961E-02/2, 0.00000000E+00, -1.39289961E-02/2, 1.96853655E+00/2, -9.75744933E-01/2,
    1.39289961E-02/2, 0.00000000E+00, -1.39289961E-02/2, 1.97619430E+00/2, -9.80729733E-01/2
};

q31_t fixed_filter_coeffs[STAGES*5];
arm_biquad_cas_df1_32x64_ins_q31 S;
static q63_t qState[STAGES*4]={0};
bool connect_flag = false;
bool wifi_connect_flag = false;
arm_rfft_instance_q31 realInstance;


// Start ADCDMA
class ADCDMA
{
    public:
        ADCDMA(int pin, uint16_t *buf, size_t bufSize);
        virtual ~ADCDMA();

        void start(size_t freqHZ);
        void stop();

    private:
        int pin;
        uint16_t *buf;
        size_t bufSize;
};

ADCDMA::ADCDMA(int pin, uint16_t *buf, size_t bufSize) : pin(pin), buf(buf), bufSize(bufSize)
{}

ADCDMA::~ADCDMA()
{}

void ADCDMA::start(size_t freqHZ)
{
    // Using Dual ADC Regular Simultaneous DMA Mode 1

    // Using Timer3. To change timers, make sure you edit all of:
    // RCC_APB1Periph_TIM3, TIM3, ADC_ExternalTrigConv_T3_TRGO

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

    // Set the pin as analog input
    // GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    // GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    HAL_Pin_Mode(pin, AN_INPUT);

    // Enable the DMA Stream IRQ Channel
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 60000000UL = 60 MHz Timer Clock = HCLK / 2
    // Even low audio rates like 8000 Hz will fit in a 16-bit counter with no prescaler (period = 7500)
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = (60000000UL / freqHZ) - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T3_TRGO
    TIM_Cmd(TIM3, ENABLE);

    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    // DMA2 Stream0 channel0 configuration
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)buf;
    DMA_InitStructure.DMA_PeripheralBaseAddr =  0x40012308; // CDR_ADDRESS; Packed ADC1, ADC2;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = bufSize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);

    // Don't enable DMA Stream Half / Transfer Complete interrupt
    // Since we want to write out of loop anyway, there's no real advantage to using the interrupt, and as
    // far as I can tell, you can't set the interrupt handler for DMA2_Stream0 without modifying
    // system firmware because there's no built-in handler for it.
    // DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);

    DMA_Cmd(DMA2_Stream0, ENABLE);

    // ADC Common Init
    ADC_CommonInitStructure.ADC_Mode = ADC_DualMode_RegSimult;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ADC1 configuration
    // 12 BIT
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Left;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // ADC2 configuration - same
    ADC_Init(ADC2, &ADC_InitStructure);

    //
    ADC_RegularChannelConfig(ADC1, PIN_MAP[pin].adc_channel, 1, ADC_SampleTime_15Cycles);
    ADC_RegularChannelConfig(ADC2, PIN_MAP[pin].adc_channel, 1, ADC_SampleTime_15Cycles);

    // Enable DMA request after last transfer (Multi-ADC mode)
    ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

    // Enable ADCs
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);

    ADC_SoftwareStartConv(ADC1);
}

void ADCDMA::stop()
{
    // Stop the ADC
    ADC_Cmd(ADC1, DISABLE);
    ADC_Cmd(ADC2, DISABLE);

    DMA_Cmd(DMA2_Stream0, DISABLE);

    // Stop the timer
    TIM_Cmd(TIM3, DISABLE);
}

ADCDMA adcDMA(SAMPLE_PIN, samples, SAMPLE_BUF_SIZE);
// End ADCDMA

void setup()
{
    Serial.begin(9600);
    System.on(button_click, buttonHandler);
    pinMode(D7, OUTPUT);
    pinMode(D3, OUTPUT);

    //Converting float to q31
    arm_float_to_q31(filter_coeffs, fixed_filter_coeffs, (STAGES*5));
    //Initializing the biquad structure with the coefficients
    arm_biquad_cas_df1_32x64_init_q31(&S, STAGES, fixed_filter_coeffs, qState, 1);

    //Initializing the fft function
    arm_rfft_init_q31(&realInstance, SAMPLE_BUF_SIZE/4, 0, 1);

    WiFi.off();
}

q31_t FftBuf[SAMPLE_BUF_SIZE/2];
q31_t SrcBuf[SAMPLE_BUF_SIZE/4];
q31_t DstBuf[SAMPLE_BUF_SIZE/4];
//char buff[20];
char buff[5];
uint32_t truck_count = 0;
uint32_t no_truck_count = 0;
uint32_t correct_truck_count = 0;
static uint32_t numOfWrites = 0;
uint8_t idx = 0;

void loop()
{
    uint16_t *sendBuf = NULL;
    uint16_t *tmpBuf = NULL;

    switch(state)
    {
        case STATE_WAITING:
            break;

        case STATE_CONNECT:
                digitalWrite(D7, HIGH);
                Serial.println("starting");
                adcDMA.start(SAMPLE_RATE);
                recordingStart = millis();
                state = STATE_RUNNING;
            break;

        case STATE_RUNNING:
            if (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_HTIF0))
            {
                DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_HTIF0);
                sendBuf = samples;
                tmpBuf = samples;
            }
            if (DMA_GetFlagStatus(DMA2_Stream0, DMA_FLAG_TCIF0))
            {
                DMA_ClearFlag(DMA2_Stream0, DMA_FLAG_TCIF0);
                sendBuf = &samples[SAMPLE_BUF_SIZE / 2];
                tmpBuf = &samples[SAMPLE_BUF_SIZE / 2];
            }

            if (sendBuf != NULL)
            {
                // There is a sample buffer to send
                // Average the pairs of samples
                for(size_t ii = 0, jj = 0; ii < SAMPLE_BUF_SIZE / 2; ii += 2, jj++)
                {
                    uint32_t sum = (uint32_t)sendBuf[ii] + (uint32_t)sendBuf[ii + 1];
                    sendBuf[jj] = (uint16_t)(sum / 2);

                    uint32_t sum1 = (uint32_t)tmpBuf[ii] + (uint32_t)tmpBuf[ii + 1];
                    tmpBuf[jj] = (uint16_t)(sum1 / 2);
                }

                //Shift adjustment to make the input of q31_t type.
                for(int x = 0; x < SAMPLE_BUF_SIZE/4; x++)
                {
                    SrcBuf[x] = (((int32_t)tmpBuf[x])-(1 << 15)) * (1 << 16);
                }

                //Send signal to filter
                arm_biquad_cas_df1_32x64_q31(&S, SrcBuf, SrcBuf, SAMPLE_BUF_SIZE/4);

                //FFT on filtered data
                arm_rfft_q31(&realInstance, SrcBuf, FftBuf);
                //Scaling the fft back to the right values
                for(int i = 0; i < SAMPLE_BUF_SIZE/4; i++)
                    FftBuf[i] = FftBuf[i] << 9;

                arm_cmplx_mag_q31(FftBuf, SrcBuf, SAMPLE_BUF_SIZE/4);

                if(correct_truck_count < 3)
                {
                    if(decisionTree())
                        truck_count++;
                    else
                        no_truck_count++;

                    //For collecting training data
                    //for(int x = 0; x < SAMPLE_BUF_SIZE/4; x++)
                    //{
                    //    sprintf(buff, "%d ", SrcBuf[x]);
                    //    client.print(buff);
                    //    if(x == ((SAMPLE_BUF_SIZE/4)-1))
                    //        client.print("\n\n\n");
                    //}
                    if (millis() - recordingStart >= MAX_RECORDING_LENGTH_MS)
                    {
                        if(truck_count >= no_truck_count)
                        {
                            //Set LED Green
                            Serial.printf("TRUCK\n");
                            RGB.control(true);
                            RGB.color(0, 255, 0);
                            correct_truck_count++;
                            recordingStart = millis();
                        }
                        else
                        {
                            Serial.printf("NO_TRUCK\n");
                            RGB.color(0, 0, 0);
                            RGB.control(false);
                            correct_truck_count = 0;
                            recordingStart = millis();
                        }
                        truck_count = 0;
                        no_truck_count = 0;
                    }
                }

                //Classify and send to cloud if necessary (Note: Send Original data)
                if(correct_truck_count >= 3)
                {
                    //Wifi on
                    if(!wifi_connect_flag)
                    {
                        Serial.printf("Before wifi connect\n");
                        WiFi.connect();
                        if(!WiFi.ready())
                            wifi_connect_flag = false;
                        else
                            wifi_connect_flag = true;
                        Serial.printf("After wifi connect\n");
                    }
                    if(!connect_flag && wifi_connect_flag)
                    {
                        Serial.printf("Before server connect\n");
                        client.connect(serverAddr, serverPort);
                        if(!client.connected())
                        {
                            connect_flag = false;
                            Serial.printf("Failed to connect\n");
                        }
                        else
                            connect_flag = true;
                    }
                    if(connect_flag)
                    {
                        Serial.printf("numOfWrites: %d\n", numOfWrites);
                        int count = client.write((uint8_t *)sendBuf, SAMPLE_BUF_SIZE / 2);
                        if (count == SAMPLE_BUF_SIZE / 2)
                        {
                            // Success
                            numOfWrites++;
                        }
                        else
                            if (count == -16)
                            {
                                // TCP Buffer full
                                Serial.printlnf("buffer full, discarding");
                                //
                            }
                            else
                            {
                                // Error
                                Serial.printlnf("error writing %d", count);
                                state = STATE_FINISH;
                            }
                    }
                }

                if(numOfWrites > 100)
                {
                    Serial.println("In server response check");
                    while (client.available() && idx < sizeof(buff))
                    {
                        buff[idx++] = client.read();
                    }
                    if(strncmp(buff, "yes", 3) == 0)
                    {
                        Serial.println("Received");
                        digitalWrite(D3, HIGH);
                        client.flush();
                        client.print("");
                        client.stop();
                        WiFi.off();
                        memset(buff, 0, 5);
                        connect_flag = false;
                        wifi_connect_flag = false;
                        correct_truck_count = 0;
                        numOfWrites = 0;
                        idx = 0;
                        delay(1000);
                        digitalWrite(D3, LOW);
                    }
                }
            }
            break;

        case STATE_FINISH:
            adcDMA.stop();
            state = STATE_WAITING;
            Serial.println("stopping");
            digitalWrite(D7, LOW);
            RGB.color(0, 0, 0);
            RGB.control(false);
            break;
    }
}

void buttonHandler(system_event_t event, int data)
{
    switch(state)
    {
        case STATE_WAITING:
            state = STATE_CONNECT;
            break;

        case STATE_RUNNING:
            state = STATE_FINISH;
            break;
    }
}

bool decisionTree(){
    if (SrcBuf[454] <= 1806438.0){
        if (SrcBuf[1017] <= 594269.0){
            if (SrcBuf[5] <= 11917828.0){
                if (SrcBuf[1011] <= 338342.0){
                    return true;
                }
                else{           return false;
                }
            }
            else{         if (SrcBuf[432] <= 8258550.0){
                return false;
            }
            else{           return true;
            }
            }
        }
        else{       if (SrcBuf[35] <= 183766752.0){
            if (SrcBuf[1009] <= 6526256.0){
                if (SrcBuf[165] <= 2301528.0){
                    return true;
                }
                else{             return false;
                }
            }
            else{           return true;
            }
        }
        else{         if (SrcBuf[767] <= 846928256.0){
            return false;
        }
        else{           return true;
        }
        }
        }
    }
    else{     if (SrcBuf[20] <= 295264576.0){
        if (SrcBuf[19] <= 905964032.0){
            if (SrcBuf[15] <= 1142962176.0){
                if (SrcBuf[14] <= 1211630848.0){
                    if (SrcBuf[490] <= 110981632.0){
                        if (SrcBuf[2] <= 342432512.0){
                            return false;
                        }
                        else{                 return false;
                        }
                    }
                    else{               return true;
                    }
                }
                else{             if (SrcBuf[971] <= 20938242.0){
                    return true;
                }
                else{               return false;
                }
                }
            }
            else{           return true;
            }
        }
        else{         return true;
        }
    }
    else{       if (SrcBuf[18] <= 1157625216.0){
        if (SrcBuf[704] <= 161015424.0){
            if (SrcBuf[517] <= 928048128.0){
                if (SrcBuf[8] <= 228687968.0){
                    return false;
                }
                else{               if (SrcBuf[673] <= 108566.0){
                    return true;
                }
                else{                 return true;
                }
                }
            }
            else{             return false;
            }
        }
        else{           return false;
        }
    }
    else{         if (SrcBuf[1019] <= 444123.0){
        return true;
    }
    else{           if (SrcBuf[17] <= 499327968.0){
        return true;
    }
    else{             return false;
    }
    }
    }
    }
    }
}
