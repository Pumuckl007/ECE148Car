def goTo(self, destination):

        LEFT = <calibrated left value>
        RIGHT = <calibrated right value>
        STRAIGHT = <calibrated straight value>
        GO = <calibrated GO value>
        STOP = <calibrated stop value>

        STEER_CNL = 1
        THROT_CNL = 2
        from donkeycar.parts.actuator import PCA9685
        S_THRESHOLD = 5 #degrees
        steerer = PCA9685(STEER_CNL)
        G_THRESHOLD = 50 #cm
        gas = PCA9685(THROT_CNL)

        while (True):
            #BEGIN STEERING SECTION
            currentAngle = get angle from IMU - 180
            currentLocation = get coordinates from GPS
            #order of params is important
            correctAngle = getAngle(currentLocation, destination) - 180
            #if we're more off to the right than S_THRESHOLD
            if currentAngle - correctAngle > S_THRESHOLD:
                steerer.run(LEFT)
            else if currentAngle - correctAngle < -S_THRESHOLD:
                steerer.run(RIGHT)
            else:
                steerer.run(STRAIGHT)
            #BEGIN THROTTLE SECTION
            left_ultra, right_ultra = get the ultrasonic vals

            if left_ultra < G_THRESHOLD or right_ultra < G_THRESHOLD:
                gas.run(STOP)
            else:
                gas.run(GO)

