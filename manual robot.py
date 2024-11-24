# generated by mBlock5 for CyberPi
# codes make you happy

import event, time, cyberpi, gamepad, mbot2

@event.start
def on_start():
    while True:
      if gamepad.is_key_pressed('L1'):
        mbot2.servo_set(70,"S3")
        mbot2.servo_set(137,"S4")

      else:
        if gamepad.is_key_pressed('R1'):
          mbot2.servo_set(180,"S3")
          mbot2.servo_set(27,"S4")

        else:
          if gamepad.is_key_pressed('Up'):
            mbot2.motor_set(20,"M2")

          else:
            if gamepad.is_key_pressed('Down'):
              mbot2.motor_set(-20,"M2")

            else:
              mbot2.motor_stop("M2")

@event.start
def on_start1():
    while True:
      if int(gamepad.get_joystick('Lx') / 3.3) > 1 and gamepad.is_key_pressed('R1'):
        mbot2.EM_set_speed(100,"EM1")
        mbot2.EM_set_speed((100 - int(gamepad.get_joystick('Lx') / 3.3)),"EM2")

      else:
        if int(gamepad.get_joystick('Lx') / 3.3) < -1 and gamepad.is_key_pressed('R1'):
          mbot2.EM_set_speed((100 - int(gamepad.get_joystick('Lx') / 3.3) * -1),"EM1")
          mbot2.EM_set_speed(100,"EM2")

        else:
          if int(gamepad.get_joystick('Lx') / 3.3) > 1 and gamepad.is_key_pressed('L1'):
            mbot2.EM_set_speed((-100 + int(gamepad.get_joystick('Rx') / 3.3)),"EM1")
            mbot2.EM_set_speed(-100,"EM2")

          else:
            if int(gamepad.get_joystick('Lx') / 3.3) < -1 and gamepad.is_key_pressed('L1'):
              mbot2.EM_set_speed(-100,"EM1")
              mbot2.EM_set_speed(((-100 + int(gamepad.get_joystick('Rx') / 3.3))) * -1,"EM2")

            else:
              if gamepad.is_key_pressed('R1'):
                mbot2.EM_set_speed(100,"EM1")
                mbot2.EM_set_speed(100,"EM2")

              else:
                if gamepad.is_key_pressed('L1'):
                  mbot2.EM_set_speed(-100,"EM1")
                  mbot2.EM_set_speed(-100,"EM2")

                else:
                  mbot2.EM_stop("ALL")