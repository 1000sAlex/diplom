
	Kalman kalmanX; // Create the Kalman instances
	Kalman kalmanY;

	/* IMU Data */
	double accX, accY, accZ;
	double gyroX, gyroY, gyroZ;

	double gyroXangle, gyroYangle; // Angle calculate using the gyro only
	double compAngleX, compAngleY; // Calculated angle using a complementary filter
	double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

	

//считаем стартовый угол по акселерометру
	  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;

	  kalmanX.setAngle(roll); // Set starting angle
	  kalmanY.setAngle(pitch);
	  gyroXangle = roll;
	  gyroYangle = pitch;
	  compAngleX = roll;
	  compAngleY = pitch;

	}

	void loop() {

	  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
//ролл и питч мы считаем на основе акселерометра
	  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
//гироскоп мы просто интегрируем	  
	  double gyroXrate = gyroX / 131.0; // Convert to deg/s
	  double gyroYrate = gyroY / 131.0; // Convert to deg/s

	  
	  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
		kalmanY.setAngle(pitch);
		compAngleY = pitch;
		kalAngleY = pitch;
		gyroYangle = pitch;
	  } else
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

	  if (abs(kalAngleY) > 90)
		gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
	  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

//вроде защита от выхода за границы на гироскопе	
	  // Reset the gyro angle when it has drifted too much
	  if (gyroXangle < -180 || gyroXangle > 180)
		gyroXangle = kalAngleX;
	  if (gyroYangle < -180 || gyroYangle > 180)
		gyroYangle = kalAngleY;


//вывод
	  Serial.print(roll); Serial.print("\t");
	  Serial.print(gyroXangle); Serial.print("\t");
	  Serial.print(compAngleX); Serial.print("\t");
	  Serial.print(kalAngleX); Serial.print("\t");

	  Serial.print("\t");

	  Serial.print(pitch); Serial.print("\t");
	  Serial.print(gyroYangle); Serial.print("\t");
	  Serial.print(compAngleY); Serial.print("\t");
	  Serial.print(kalAngleY); Serial.print("\t");

	}
