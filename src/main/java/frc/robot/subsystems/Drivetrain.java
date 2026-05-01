package frc.robot.subsystems;

// Gerekli REV Robotics ve WPILib kütüphanelerini importluyoruz.
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Port numaralarının (ID) ve limitlerin tutulduğu Constants dosyasını çağırıyoruz.
import static frc.robot.Constants.DriveConstants.*;

public class Drivetrain extends SubsystemBase {
    
    // 1. NEO MOTORLARI TANIMLAMA 
    private final SparkMax solAnaMotor;
    private final SparkMax solTakipciMotor;
    private final SparkMax sagAnaMotor;
    private final SparkMax sagTakipciMotor;

    // WPILib'in tank sürüşünü sağladıüı classı
    private final DifferentialDrive surusSistemi;

    public Drivetrain() {
        // 2. MOTOR OBJELERİNİ OLUŞTURMA
        // NEO motorlar fırçasız olduğundan MotorType.kBrushless kullanıyoruz
        solAnaMotor = new SparkMax(LEFT_LEADER_ID, MotorType.kBrushless);
        solTakipciMotor = new SparkMax(LEFT_FOLLOWER_ID, MotorType.kBrushless);
        sagAnaMotor = new SparkMax(RIGHT_LEADER_ID, MotorType.kBrushless);
        sagTakipciMotor = new SparkMax(RIGHT_FOLLOWER_ID, MotorType.kBrushless);

        // 3. SÜRÜŞ SİSTEMİNİ KURMA
        // Sadece ana motorları diferansiyel sürüşe veriyoruz.
        surusSistemi = new DifferentialDrive(solAnaMotor, sagAnaMotor);

        // 4. NEO MOTOR AYAR ŞABLONU (CONFIG) OLUŞTURMA
        SparkMaxConfig ayarlar = new SparkMaxConfig();
        
        // Voltaj dengeleme ve akım sınırı: NEO'lar çok güç çekebileceği için amper sınırlamaları koyuyruz
        ayarlar.voltageCompensation(12); 
        ayarlar.smartCurrentLimit(DRIVE_MOTOR_CURRENT_LIMIT); 
        
        // Boşta Durum Modu: Kumanda bırakıldığında robot ani fren yapsım
        ayarlar.idleMode(IdleMode.kBrake);

        // 5. TAKİPÇİ (FOLLOWER) MOTORLARI AYARLAMA
        ayarlar.follow(solAnaMotor);
        solTakipciMotor.configure(ayarlar, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        ayarlar.follow(sagAnaMotor);
        sagTakipciMotor.configure(ayarlar, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // 6. ANA MOTORLARI AYARLAMA VE YÖN TERSLEME
        ayarlar.disableFollowerMode();
        sagAnaMotor.configure(ayarlar, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // Şasenin kendi etrafında dönmemesi için sol tarafı ters çeviriyoruz
        ayarlar.inverted(true);
        solAnaMotor.configure(ayarlar, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        //Otonom kodunu yazarken burayı kullanacağız.
    }

    // 7. SÜRÜŞ METODU
    /**
     * Robotu kumanda ile kontrol etmemizi sağlayan temel fonksiyon.
     * "ileriGeri" İleri veya geri gitme hızı (-1.0 ile 1.0 arası)
     * "donus" Sağa veya sola dönme hızı (-1.0 ile 1.0 arası)
     */
    public void arcadeSurus(double ileriGeri, double donus) {
        surusSistemi.arcadeDrive(ileriGeri, donus);
    }
}