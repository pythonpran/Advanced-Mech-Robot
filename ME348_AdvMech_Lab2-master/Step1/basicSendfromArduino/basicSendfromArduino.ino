  
void setup() {
  delay(10000);
    Serial.begin(115200);
    Serial.println("resetting arduino");
 
}   

void loop() {
  Serial.println("<Arduino is Ready>");
  delay(1000);

}
