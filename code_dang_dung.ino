#include <REG.h>
#include <wit_c_sdk.h>
#include <arduinoFFT.h>
#include <Fast4ier.h>
#include <complex.h>
#include "rotate.h"
#include "BluetoothSerial.h"
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

complex j(0,1);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
BluetoothSerial SerialBT;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
	WitInit(WIT_PROTOCOL_NORMAL, 0x50);
	WitSerialWriteRegister(SensorUartSend);
	WitRegisterCallBack(SensorDataUpdata);
  WitDelayMsRegister(Delayms);
	Serial.print("\r\n********************** wit-motion normal example  ************************\r\n");
	AutoScanSensor();
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  ////
}
/*int i;
int count=0;
float fAcc[3], fGyro[3], fAngle[3];*/
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
    ///////////////////////////
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
  complex A[sample];
  for(int i=0;i<sample;i++)
  {
     A[i]= complex(vReal[i],0);
  }
  Serial.print("\n");
  Fast4::FFT(A,sample);//IFFT has similar usage. this is the inplace version. for out of place use: Fast4::FFT(input data as complec[], output buffer as complex, the number of bins); //also can be IFFT.
  Serial.println(F("##########################################"));

  //delay(5000);
 /* FFT.Windowing(vReal, sample, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, sample, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, sample);
  float peak = FFT.MajorPeak(vReal, sample,  sample_rate);
   Serial.print("Detected Frequency: "); 
   Serial.print(String(peak));
   Serial.println(" Hz");*/
    Serial.print("gia toc sau khi FFT");
 for(int i=0;i<sample;i++)
  {
  Serial.print(A[i].re());
  Serial.print(F(" + "));
  Serial.print(A[i].im());
  Serial.println(F("i"));
  } 
 /* delay(1000);
  complex V[sample];
  complex D[sample];
  complex w(2*PI*10,0);
  complex j(0,1);
  for(int i=0;i<sample;i++)
  {
  V[i]= A[i]/(j*w);
  D[i]= A[i]/(0-(w*w));
  }
  Serial.print("thong so van toc chua  IFFT");
   for(int i=0;i<sample;i++)
  {
  Serial.print(V[i].re());
  Serial.print(F(" + "));
  Serial.print(V[i].im());
  Serial.println(F("i"));
  }
  Serial.print("thong so vi tri chua IFFT");
  for(int i=0;i<sample;i++)
  {
  Serial.print(D[i].re());
  Serial.print(F(" + "));
  Serial.print(D[i].im());
  Serial.println(F("i"));
  }
  delay(1000);
  Fast4::IFFT(V,sample);//IFFT has similar usage. this is the inplace version. for out of place use: Fast4::FFT(input data as complec[], output buffer as complex, the number of bins); //also can be IFFT.
  Serial.println(F("##########################################"));
  Serial.print("thong so van toc sau khi IFFT");
 for(int i=0;i<sample;i++)
  {
  Serial.print(V[i].re());
  Serial.print(F(" + "));
  Serial.print(V[i].im());
  Serial.println(F("i"));
  }

Fast4::IFFT(D,sample);//IFFT has similar usage. this is the inplace version. for out of place use: Fast4::FFT(input data as complec[], output buffer as complex, the number of bins); //also can be IFFT.
  Serial.println(F("##########################################"));
  Serial.print("thong so vi tri sau khi IFFT");
 for(int i=0;i<sample;i++)
  {
  Serial.print(D[i].re());
  Serial.print(F(" + "));
  Serial.print(D[i].im());
  Serial.println(F("i"));
  }
  delay(1000);
  count=0;*/
}






