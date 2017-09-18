

class DistanceSensor {
  private:
    int trigPin1;
    int echoPin1;
    int trigPin2;
    int echoPin2;
    float last_distance_cm;
    long lastTime_ms;
    int temperature_in_c = 20;
  public:
    void setup(int trigPin1, int echoPin1, int trigPin2, int echoPin2 ) {
      this->trigPin1 = trigPin1;
      this->echoPin1 = echoPin1;
      this->trigPin2 = trigPin2;
      this->echoPin2 = echoPin2;
      pinMode(trigPin1, OUTPUT);
      pinMode(echoPin1, INPUT);
      pinMode(trigPin2, OUTPUT);
      pinMode(echoPin2, INPUT);
      last_distance_cm = 0;
      digitalWrite(trigPin1, LOW);
      digitalWrite(trigPin2, LOW);
    }

    long doPulseAndMeasureTime_us(int trigPin, int echoPin) {
      long duration;
 
      digitalWrite(trigPin, HIGH);
      delayMicroseconds((trigPin==trigPin2) ? 10 : 10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH, 60000 /* wait up to 60ms for return pulse */ );
      // pulseIn will only return 0 if it timed out. (or if echoPin was already to 1, but it should not happen)
      /*if(duration == 0) // If we timed out
      {
        pinMode(echoPin, OUTPUT); // Then we set echo pin to output mode
        digitalWrite(echoPin, LOW); // We send a LOW pulse to the echo pin
        delayMicroseconds(200);
        pinMode(echoPin, INPUT); // And finaly we come back to input mode
      }*/
      return duration;
    }
  
    float getDistance_cm(bool &ok) {
      long now_ms = millis();
      int diff = now_ms - lastTime_ms;
      if (diff < 60) {
        delay(60 - diff); // Wait for previous echo to finish
        lastTime_ms = now_ms = millis();
      }
      const float speed_of_sound = 331.3 + 0.606 * temperature_in_c;

      long duration_us = doPulseAndMeasureTime_us(trigPin1, echoPin1);

      long duration2_us = 0;
      if(duration_us != 0) {
        unsigned long startPulse_us = micros();
        duration2_us = doPulseAndMeasureTime_us(trigPin2, echoPin2);
        duration2_us = micros() - startPulse_us;
      }

      float distance1_cm = (duration_us/2.0) * speed_of_sound / 10000;

      float distance2_cm = 0;
      if(duration2_us != 0)
        distance2_cm = distance1_cm + ((duration2_us)/2.0) * speed_of_sound / 10000;
//        distance2_cm = distance1_cm + ((duration2_us-448)/2.0) * speed_of_sound / 10000;

      Serial.print("distance2:");
      Serial.print(distance2_cm );
      Serial.print("   ");
    
      ok = (distance1_cm < 200 && distance1_cm > 0);
      if(!ok)
        distance1_cm = last_distance_cm;
      else
        last_distance_cm = distance1_cm;
       return distance1_cm;
    }
};


