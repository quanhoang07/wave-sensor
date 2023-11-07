#include <REG.h>
#include <wit_c_sdk.h>
#include <arduinoFFT.h>
#include <Fast4ier.h>
#include <complex.h>
#define sample 16
#define sample_rate 1000
double vReal[sample];
double vImag[sample];
arduinoFFT FFT = arduinoFFT();
/*
Test on MEGA 2560. use WT901CTTL sensor

WT901CTTL     MEGA 2560
    VCC <--->  5V/3.3V
    TX  <--->  19(TX1)
    RX  <--->  18(RX1)
    GND <--->  GND
*/

#define ACC_UPDATE		0x01
#define GYRO_UPDATE		0x02
#define ANGLE_UPDATE	0x04
#define MAG_UPDATE		0x08
#define READ_UPDATE		0x80
static volatile char s_cDataUpdate = 0, s_cCmd = 0xff; 

static void CmdProcess(void);
static void AutoScanSensor(void);
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize);
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
static void Delayms(uint16_t ucMs);
const uint32_t c_uiBaud[8] = {0,4800, 9600, 19200, 38400, 57600, 115200, 230400};
//////////////////////

#define M_PI 3.14159265358979323846;
//static void caculate(float rawData_X, float rawData_Y,float rawData_Z);
 void RotationMatrix(float mtR[3][3], float mtP[3][3], float mtY[3][3], float result[1][3], float mtgoc[1][3]);
 void calRoll(float x, float mtR[3][3]);
 void calPitch(float x, float mtP[3][3]);
 void calYaw(float x,float y, float mtY[3][3]);
 void Calculate(float result[1][3]);
float mtR[3][3], mtP[3][3], mtY[3][3], result[3];
void Cal(double result[3]);

    ////////////////

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
	Serial.print("\r\n********************** wit-motion normal example  ************************\r\n");
	AutoScanSensor();

  ////
}
int i;
int count=0;
float fAcc[3], fGyro[3], fAngle[3];
void loop() {
  
  do
  {
    while (Serial2.available())
    {
      WitSerialDataIn(Serial2.read());
    }
    while (Serial.available()) 
    {
      CopeCmdData(Serial.read());
    }
		CmdProcess();
		if(s_cDataUpdate)
		{
			for(i = 0; i < 3; i++)
			{
				fAcc[i] = sReg[AX+i] / 32768.0f * 16.0f;
				fGyro[i] = sReg[GX+i] / 32768.0f * 2000.0f;
				fAngle[i] = sReg[Roll+i] / 32768.0f * 180.0f;
			}
			if(s_cDataUpdate & ACC_UPDATE)
			{
				Serial.print("acc:");
				Serial.print(fAcc[0], 3);
				Serial.print(" ");
				Serial.print(fAcc[1], 3);
				Serial.print(" ");
				Serial.print(fAcc[2], 3);
				Serial.print("\r\n");
				s_cDataUpdate &= ~ACC_UPDATE;
			}

			if(s_cDataUpdate & ANGLE_UPDATE)
			{
				Serial.print("angle:");
				Serial.print(fAngle[0], 3);
				Serial.print(" ");
				Serial.print(fAngle[1], 3);
				Serial.print(" ");
				Serial.print(fAngle[2], 3);
				Serial.print("\r\n");
				s_cDataUpdate &= ~ANGLE_UPDATE;
			}

      s_cDataUpdate = 0;
		}
    Cal(result);
    
    ///////////////////////////////
    vReal[count]=result[2];
    Serial.print("vReal: ");
    Serial.println(vReal[count], 3);
    //delay(1000);
    count++;
  }while(count < sample);

  for(int i=0;i<sample;i++) {
    Serial.print(vReal[i], 3);
    Serial.print(" ");
  }
  complex vals[sample];
  for(int i=0;i<sample;i++)
  {
     vals[i]= complex(vReal[i],0);
  }
  Serial.print("\n");
  Fast4::FFT(vals,sample);//IFFT has similar usage. this is the inplace version. for out of place use: Fast4::FFT(input data as complec[], output buffer as complex, the number of bins); //also can be IFFT.
  Serial.println(F("##########################################"));

  //delay(5000);
 /* FFT.Windowing(vReal, sample, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, sample, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, sample);
  float peak = FFT.MajorPeak(vReal, sample,  sample_rate);
   Serial.print("Detected Frequency: "); 
   Serial.print(String(peak));
   Serial.println(" Hz");*/
 for(int i=0;i<sample;i++)
{
  Serial.print(vals[i].re());
  Serial.print(F(" + "));
  Serial.print(vals[i].im());
  Serial.println(F("i"));
}
  delay(1000);
  Fast4::IFFT(vals,sample);//IFFT has similar usage. this is the inplace version. for out of place use: Fast4::FFT(input data as complec[], output buffer as complex, the number of bins); //also can be IFFT.
  Serial.println(F("##########################################"));
 for(int i=0;i<sample;i++)
{
  Serial.print(vals[i].re());
  Serial.print(F(" + "));
  Serial.print(vals[i].im());
  Serial.println(F("i"));
}
  delay(1000);
  count=0;
}


void CopeCmdData(unsigned char ucData)
{
	static unsigned char s_ucData[50], s_ucRxCnt = 0;
	
	s_ucData[s_ucRxCnt++] = ucData;
	if(s_ucRxCnt<3)return;										//Less than three data returned
	if(s_ucRxCnt >= 50) s_ucRxCnt = 0;
	if(s_ucRxCnt >= 3)
	{
		if((s_ucData[1] == '\r') && (s_ucData[2] == '\n'))
		{
			s_cCmd = s_ucData[0];
			memset(s_ucData,0,50);
			s_ucRxCnt = 0;
		}
		else 
		{
			s_ucData[0] = s_ucData[1];
			s_ucData[1] = s_ucData[2];
			s_ucRxCnt = 2;
			
		}
	}
}
static void ShowHelp(void)
{
	Serial.print("\r\n************************	 WIT_SDK_DEMO	************************");
	Serial.print("\r\n************************          HELP           ************************\r\n");
	Serial.print("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	Serial.print("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	Serial.print("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	Serial.print("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	Serial.print("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	Serial.print("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	Serial.print("UART SEND:R\\r\\n   The return rate increases to 10Hz.\r\n");
  Serial.print("UART SEND:r\\r\\n   The return rate reduction to 1Hz.\r\n");
  Serial.print("UART SEND:C\\r\\n   Basic return content: acceleration, angular velocity, angle, magnetic field.\r\n");
  Serial.print("UART SEND:c\\r\\n   Return content: acceleration.\r\n");
  Serial.print("UART SEND:h\\r\\n   help.\r\n");
	Serial.print("******************************************************************************\r\n");
}

static void CmdProcess(void)
{
	switch(s_cCmd)
	{
		case 'a':	if(WitStartAccCali() != WIT_HAL_OK) Serial.print("\r\nSet AccCali Error\r\n");
			break;
		case 'm':	if(WitStartMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
			break;
		case 'e':	if(WitStopMagCali() != WIT_HAL_OK) Serial.print("\r\nSet MagCali Error\r\n");
			break;
		case 'u':	if(WitSetBandwidth(BANDWIDTH_5HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
			break;
		case 'U':	if(WitSetBandwidth(BANDWIDTH_256HZ) != WIT_HAL_OK) Serial.print("\r\nSet Bandwidth Error\r\n");
			break;
		case 'B':	if(WitSetUartBaud(WIT_BAUD_115200) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else 
              {
                Serial2.begin(c_uiBaud[WIT_BAUD_115200]);
                Serial.print(" 115200 Baud rate modified successfully\r\n");
              }
			break;
		case 'b':	if(WitSetUartBaud(WIT_BAUD_9600) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else 
              {
                Serial2.begin(c_uiBaud[WIT_BAUD_9600]); 
                Serial.print(" 9600 Baud rate modified successfully\r\n");
              }
			break;
		case 'r': if(WitSetOutputRate(RRATE_1HZ) != WIT_HAL_OK)  Serial.print("\r\nSet Baud Error\r\n");
			        else Serial.print("\r\nSet Baud Success\r\n");
			break;
		case 'R':	if(WitSetOutputRate(RRATE_10HZ) != WIT_HAL_OK) Serial.print("\r\nSet Baud Error\r\n");
              else Serial.print("\r\nSet Baud Success\r\n");
			break;
    case 'C': if(WitSetContent(RSW_ACC|RSW_GYRO|RSW_ANGLE|RSW_MAG) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
    case 'c': if(WitSetContent(RSW_ACC) != WIT_HAL_OK) Serial.print("\r\nSet RSW Error\r\n");
      break;
		case 'h':	ShowHelp();
			break;
		default :break;
	}
	s_cCmd = 0xff;
}
static void SensorUartSend(uint8_t *p_data, uint32_t uiSize)
{
  Serial2.write(p_data, uiSize);
  Serial2.flush();
}
static void Delayms(uint16_t ucMs)
{
  delay(ucMs);
}
static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < sizeof(c_uiBaud)/sizeof(c_uiBaud[0]); i++)
	{
		Serial2.begin(c_uiBaud[i]);
    Serial2.flush();
		iRetry = 2;
		s_cDataUpdate = 0;
		do
		{
			WitReadReg(AX, 3);
			delay(200);
      while (Serial2.available())
      {
        WitSerialDataIn(Serial2.read());
      }
			if(s_cDataUpdate != 0)
			{
				Serial.print(c_uiBaud[i]);
				Serial.print(" baud find sensor\r\n\r\n");
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	Serial.print("can not find sensor\r\n");
	Serial.print("please check your connection\r\n");
}


 void RotationMatrix(float mtR[3][3], float mtP[3][3], float mtY[3][3], float result[3], float mtgoc[3]) {
    float temp[3][3];
    float temp1[3][3];
    // tính toán ma trận xoay
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                temp[i][j] += mtR[i][k] * mtP[k][j];
            }
        }
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            temp1[i][j] = 0;
            for (int k = 0; k < 3; k++) {
                temp1[i][j] += temp[i][k] * mtY[k][j];
            }
        }
    }
    // tinh tham so a
    for (int i = 0; i < 1; i++) {
        for (int j = 0; j < 3; j++) {
            result[j] = 0;
            for (int k = 0; k < 3; k++) {
                result[j] += mtgoc[k] * temp1[k][j];
            }
        }
    }
}
 void calRoll(float x, float mtR[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mtR[i][j] = 0;
        }
    }
    mtR[0][0] = cos(x);
    mtR[0][1] = sin(x);
    mtR[1][0] = -sin(x);
    mtR[1][1] = cos(x);
    mtR[2][2] = 1;
}
 void calPitch(float x, float mtP[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mtP[i][j] = 0;
        }
    }
    mtP[0][0] = cos(x);
    mtP[0][2] = sin(x);
    mtP[2][0] = -sin(x);
    mtP[2][2] = cos(x);
    mtP[1][1] = 1;
}
 void calYaw(float x,float y, float mtY[3][3])
{
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            mtY[i][j] = 0;
        }
    }
    mtY[1][1] = cos(x);
    mtY[2][1] = sin(x);
    mtY[1][2] = -sin(x);
    mtY[2][2] = cos(y);
    mtY[0][0] = 1;
}
void Cal(float result[3])
{
    calRoll(fAngle[0], mtR);
    calPitch(fAngle[1], mtP);
    calYaw(fAngle[2], fAngle[1], mtY);
    // Thực hiện phép nhân ma trận
    RotationMatrix(mtR, mtP, mtY, result, fAcc);
    Serial.print("Toa do GCS: ");
      for (int i = 0; i < 3; i++) 
      {
          Serial.print(result[i], 3);
          Serial.print(" ");
      }
      Serial.println("\r\n");
}
