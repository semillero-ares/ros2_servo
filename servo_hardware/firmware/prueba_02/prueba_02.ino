void setup() {
  Serial.begin(115200);
}

int i = 0;
void loop() {
  i++;
  Serial.println(i);
}
