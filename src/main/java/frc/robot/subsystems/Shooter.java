package frc.robot.subsystems;

// Gerekli REV Robotics kütüphanelerini içe aktarıyoruz
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

// Subsystem altyapısı için WPILib kütüphanesi
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Port numaralarının tutulduğu Constants dosyasını çağırıyoruz
import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {
    