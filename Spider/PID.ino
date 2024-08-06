//---------------CALCULATE PID------------------
unsigned long timeNow_pid = 0, timeLater_pid = 0, timeLater_Ti = 0, timeLater_Td = 0;
double error_pid, errorLater_pid, sumError_pid;

double cal_pid(double setpoint_pid, double inMin, double inMax,
               double outMin, double outMax, double Ti, double Td,
               double kp_pid, double ki_pid, double kd_pid, double feedback) {

  timeNow_pid = millis();
  timeLater_pid = timeNow_pid;

  unsigned long int deltaTime = timeNow_pid - timeLater_pid;
  double deltaError =  error_pid -  errorLater_pid;

  double error_pid = (setpoint_pid  - feedback) / (inMax - inMin);

  double cP = error_pid;
  double cI = sumError_pid;
  double cD = deltaError;

  double out_temp = (kp_pid * cP) + (ki_pid * cI) + (kd_pid *  cD);
  double out_pid = out_temp * (outMax - outMin);

  if (out_pid > outMax) {
    out_pid = outMax;
  } else if (out_pid < outMin) {
    out_pid = outMin;
  }

  if (timeNow_pid - timeLater_Ti > Ti) {
    timeLater_Ti = timeNow_pid;
    sumError_pid = sumError_pid + error_pid;
    if (sumError_pid > outMax) {
      sumError_pid = outMax;
    } else if (sumError_pid < outMin) {
      sumError_pid = outMin;
    }
  }

  if (timeNow_pid - timeLater_Td > Td) {
    timeLater_Td = timeNow_pid;
    errorLater_pid = error_pid;
  }

  if ( errorLater_pid * error_pid < 0 ) {
    sumError_pid = 0;
  }

  timeLater_pid = timeNow_pid;
  errorLater_pid = error_pid;
  out_pid = out_pid;

  return out_pid;
}


void scanImu() {
  computedImu  = readBNO(0);
  if (computedImu > 180) {
    computedImu = computedImu - 360;
  }
  if (imu_ref > 180) {
    computedImu - 360;
  }
  computedImu = -computedImu + imu_ref;
  if (computedImu < -180) {
    computedImu = computedImu + 360;
  }
  else if (computedImu > 180) {
    computedImu = computedImu - 360;
  }
  read_gyroy = readBNO(1) / 100.0;
}
