void OdriveInit() {

      for (int axis = 0; axis < 2; ++axis) {
          Serial2 << "w axis" << axis << ".controller.config.vel_limit " << 360000.0f << '\n';
          Serial2 << "w axis" << axis << ".motor.config.current_lim " << 30.0f << '\n';
          Serial2 << "w axis" << axis << ".motor.config.calibration_current " << 15.0f << '\n';

          //delay(1000);

          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive.run_state(axis, requested_state, true);

          delay(1000);
    
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive.run_state(axis, requested_state, true);

          delay(1000);
    
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive.run_state(axis, requested_state, false); // don't wait 
      }       
        
}

void OdriveInit2() {

      for (int axis = 0; axis < 2; ++axis) {
          Serial3 << "w axis" << axis << ".controller.config.vel_limit " << 250000.0f << '\n';
          Serial3 << "w axis" << axis << ".motor.config.current_lim " << 40.0f << '\n';
          Serial3 << "w axis" << axis << ".motor.config.calibration_current " << 15.0f << '\n';

          //delay(1000);

          requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(axis, requested_state, true);

          delay(1000);
    
          requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(axis, requested_state, true);

          delay(1000);
    
          requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(axis, requested_state, false); // don't wait 
      }       
        
}



