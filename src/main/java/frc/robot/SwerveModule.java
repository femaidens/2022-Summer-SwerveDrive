// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/** Add your docs here. */
public class SwerveModule {
  private static final double wheelRadius = 0.05; //placeholder value
  private static final int encoderResolution = 5000; //placeholder value
  private static final double kModuleMaxAngularVelocity = DriveTrain.maxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; //radians per second squared

  private final MotorController driveMotor;
  private final MotorController turningMotor;
  
  private final Encoder driveEncoder;
  private final Encoder turningEncoder;

  //placeholder kp, ki, kp values
  private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

  private final ProfiledPIDController m_turningPIDController =
    new ProfiledPIDController(1, 0, 0, new TrapezoidProfile.Constraints(5, 10)); //placeholder values
  
  //placeholder ks, kv values
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  public SwerveModule(
    int driveMotorChannel, 
    int turningMotorChannel, 
    int driveEncoderChannelA,
    int driveEncoderChannelB, 
    int turningEncoderChannelA, 
    int turningEncoderChannelB) {
      driveMotor = new PWMSparkMax(driveMotorChannel);
      turningMotor = new PWMSparkMax(turningMotorChannel);
      driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
      turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

      driveEncoder.setDistancePerPulse(2*Math.PI * wheelRadius / encoderResolution);

      turningEncoder.setDistancePerPulse(2 * Math.PI / encoderResolution);

      m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

    }

    public SwerveModuleState getState() {
      return new SwerveModuleState(driveEncoder.getRate(), new Rotation2d(turningEncoder.get()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
      SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(turningEncoder.get()));
      
      final double driveOutput = 
        m_drivePIDController.calculate(driveEncoder.getRate(), state.speedMetersPerSecond);

      final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

      final double turnOutput = 
        m_turningPIDController.calculate(turningEncoder.get(), state.angle.getRadians());

      final double turnFeedforward = 
        m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

      driveMotor.setVoltage(driveOutput + driveFeedforward);
      turningMotor.setVoltage(turnOutput + turnFeedforward);

    }
  
}
