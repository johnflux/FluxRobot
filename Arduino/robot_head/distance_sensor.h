
#define USE_SECOND_SENSOR

class DistanceSensor {
  private:
    int trigPin1;
    int echoPin1;
    int trigPin2;
    int echoPin2;
    unsigned long lastTime_ms;
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
      lastTime_ms = 0;
      digitalWrite(trigPin1, LOW);
      digitalWrite(trigPin2, LOW);
    }

    long doPulseAndMeasureTime_us(int trigPin, int echoPin) {
      long duration;

      digitalWrite(trigPin, LOW);
      delayMicroseconds(2);
      digitalWrite(trigPin, HIGH);
      delayMicroseconds(10);
      digitalWrite(trigPin, LOW);
      duration = pulseIn(echoPin, HIGH);
      // pulseIn will only return 0 if it timed out. (or if echoPin was already to 1, but it should not happen)
      if(duration == 0) // If we timed out
      {
        pinMode(echoPin, OUTPUT); // Then we set echo pin to output mode
        digitalWrite(echoPin, LOW); // We send a LOW pulse to the echo pin
        delayMicroseconds(200);
        pinMode(echoPin, INPUT); // And finaly we come back to input mode
      }
      return duration;
    }
  
    void getDistance_m(bool &ok1, float &distance1_m, bool &ok2, float &distance2_m) {
      unsigned long now_ms = millis();
      unsigned long diff = now_ms - lastTime_ms;
      if (diff < 60)
        delay(60 - diff); // Wait for previous echo to finish
      lastTime_ms = millis();
      const float speed_of_sound = 331.3 + 0.606 * temperature_in_c;


      /* Send a pulse from the first sonar and measure the time of the pulse on echoPin
       *  which tells us the time it takes to return.
       *  
       * Note that the wall clock time is going to be the duration_us + setup time.
       * 
       * For the first pulse, we aren't interested in the setup time.  But we do need to 
       * take it into account for the second echo - see next comment!       
       */
#ifdef USE_SECOND_SENSOR
      unsigned long startPulse_us = micros();
#endif      
      long duration_us = doPulseAndMeasureTime_us(trigPin1, echoPin1);
#ifdef USE_SECOND_SENSOR
      long duration2_us = 0;
      if(duration_us != 0) {
        unsigned long first_duration_plus_setup_us = micros() - startPulse_us;
        duration2_us = doPulseAndMeasureTime_us(trigPin2, echoPin2);

        /* When we listen for the second echo from the first pulse, we want to add together the time of the first
         *  echo plus the setup time for listening to the second echo plus the time reported for the second echo.
         *  
         *  If we assume that the setup for the first echo equals the setup time for the second echo, we can do:
        */
        if(duration2_us) {
          duration2_us += first_duration_plus_setup_us;
        }
      }
#endif

      distance1_m = (duration_us/2.0) * speed_of_sound / 1000000.0;

      distance2_m = 0;
#ifdef USE_SECOND_SENSOR
      if(duration2_us != 0)
        distance2_m = (duration2_us/2.0) * speed_of_sound / 1000000.0;
#endif

      ok1 = (distance1_m < 4 && distance1_m >= 0);
      ok2 = (distance2_m < 8 && distance2_m >= 0);
      if(!ok1)
        distance1_m = 0;
      if(!ok2)
        distance2_m = 0;
    }
};


