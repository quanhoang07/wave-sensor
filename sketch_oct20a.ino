#include <fftw3.h>
#include <math.h>
// macros for the real and imaginary parts
#define REAL 0
#define IMAG 1
// length of the complex arrays
#define N 8

/* Computes the 1-D fast Fourier transform. */
void fft(fftw_complex *in, fftw_complex *out)
{
	// create a DFT plan
	fftw_plan plan = fftw_plan_dft_1d(N, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
	// execute the plan
	fftw_execute(plan);
	// do some cleaning
	fftw_destroy_plan(plan);
	fftw_cleanup();
}

/* Computes the 1-D inverse fast Fourier transform. */
void ifft(fftw_complex *in, fftw_complex *out)
{
	// create an IDFT plan
	fftw_plan plan = fftw_plan_dft_1d(N, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);
	// execute the plan
	fftw_execute(plan);
	// do some cleaning
	fftw_destroy_plan(plan);
	fftw_cleanup();
	// scale the output to obtain the exact inverse
	for (int i = 0; i < N; ++i) {
		out[i][REAL] /= N;
		out[i][IMAG] /= N;
	}
}

/* Displays complex numbers in the form a +/- bi. */
void displayComplex(fftw_complex *y)
{
	for (int i = 0; i < N; ++i)
		{
      if (y[i][IMAG] < 0)
      {
			  Serial.print(y[i][REAL],3); 
        Serial.print(" - ") ;
        Serial.print(abs(y[i][IMAG]));
        Serial.println("i");
      }else
      {
				Serial.print(y[i][REAL],3); 
        Serial.print(" + ") ;
        Serial.print(abs(y[i][IMAG]));
        Serial.println("i");
      }
    }
}

/* Displays the real parts of complex numbers. */
void displayReal(fftw_complex *y)
{
	for (int i = 0; i < N; ++i)
		Serial.println(y[i][REAL]);
}

/* Test */
// input array
	fftw_complex x[N];
	// output array
	fftw_complex y[N];
	// fill the first array with some numbers
	

void setup() {
  Serial.begin(9600);
}

void loop() {
  for (int i = 0; i < N; ++i) {
		x[i][REAL] = i + 1;		// i.e., { 1, 2, 3, 4, 5, 6, 7, 8 }
		x[i][IMAG] = 0;
	}
	// compute the FFT of x and store the results in y
	fft(x, y);
	// display the results
	Serial.println("FFT =");
	displayComplex(y);
	// compute the IFFT of y and store the results in x
	ifft(y, x);
	// display the results
	Serial.println("\nIFFT =" );
	displayReal(x);
}
