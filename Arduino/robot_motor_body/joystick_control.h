

class JoystickControl {
  int joyXPin; //  X - axis of Joystick
  int joyYPin; // y - axis of Joystick
  int joyButtonPin;
  void setup(int joyXPin = A0, int joyYPin = A1, int joyButtonPin = A2) {
    this->joyXPin = joyXPin;
    this->joyYPin = joyYPin;
    this->joyButtonPin = joyButtonPin;
  }

  int joyXValue() {
    return analogRead(joyXPin);
  }
  int joyYValue() {
    return analogRead(joyYPin);
  }
  int joyButtonValue() {
    return analogRead(joyButtonPin);
  }
};
