#include <ButterFilter.h>

ButterFilter filter;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  filter.begin(0.5, 10);
  float* coeffs = filter.getCoeffs();

  Serial.print(filter.getCutoff());
  Serial.println();
  Serial.print(filter.getFs());
  Serial.println();
  for (int i = 0; i < 3; i++){
    Serial.print(coeffs[i]);
    Serial.print(" ");
  }
  Serial.println();
  
  float data[20];
  for (int i = 0; i < 10; i++)
    data[i] = 0;
  for (int i = 10; i < 20; i++)
    data[i] = 1;

  for (int i = 0; i < 20; i++){
    float filtered = filter.filter(data[i]);
    Serial.print(filtered);
    Serial.print(" ");
  }
  Serial.println();
}

void loop() {
  // put your main code here, to run repeatedly:
}
