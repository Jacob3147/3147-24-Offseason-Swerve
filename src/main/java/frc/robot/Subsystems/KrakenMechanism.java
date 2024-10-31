package frc.robot.Subsystems;




import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KrakenMechanism extends SubsystemBase 
{
    TalonFX kraken_motor = new TalonFX(20);
    TalonFXConfiguration kraken_config = new TalonFXConfiguration();

    VoltageOut kraken_voltage = new VoltageOut(0);
    PositionVoltage kraken_position = new PositionVoltage(0);

    public KrakenMechanism()
    {
        kraken_config.CurrentLimits.StatorCurrentLimit = 100;
        kraken_config.CurrentLimits.StatorCurrentLimitEnable = true;
        kraken_config.CurrentLimits.SupplyCurrentLimit = 40;
        kraken_config.CurrentLimits.SupplyCurrentLimitEnable = true;

        kraken_config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        kraken_config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        kraken_config.Slot0.kA = 0;
        kraken_config.Slot0.kV = 0;
        kraken_config.Slot0.kS = 0;
        kraken_config.Slot0.kG = 0;
        kraken_config.Slot0.kP = 1; //error of 1 rotation = 1 volt
        kraken_config.Slot0.kI = 0;
        kraken_config.Slot0.kD = 0;
        
        kraken_motor.getConfigurator().apply(kraken_config);
    }

    public void forward()
    {
        kraken_voltage.Output = 5;
        kraken_motor.setControl(kraken_voltage);
    }

    public void reverse()
    {
        kraken_voltage.Output = -5;
        kraken_motor.setControl(kraken_voltage);
    }

    public void stop()
    {
        kraken_voltage.Output = 0;
        kraken_motor.setControl(kraken_voltage);
    }


    public void seekPosition()
    {
        kraken_motor.setControl(kraken_position);
    }

    @Override
    public void periodic() 
    {
        
    }
}
