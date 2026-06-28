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

    // 1. MOTORLARI TANIMLAMA
    // Atıcı mekanizmasında genellikle tekerleri döndüren iki motor olur. Biri sağda biri solda olabilir, bu yüzden yine ana ve takipçi mantığını kullanıyoruz.
    private final SparkMax aticiAnaMotor;
    private final SparkMax aticiTakipciMotor;

    // 2. CONSTRUCTOR 
    public Shooter() {
        // Motor objelerini Fırçasız NEO motor olarak oluşturuyoruz.
        aticiAnaMotor = new SparkMax(SHOOTER_LEADER_ID, MotorType.kBrushless);
        aticiTakipciMotor = new SparkMax(SHOOTER_FOLLOWER_ID, MotorType.kBrushless);

        // 3. CONFIGRATION OLUŞTURMA
        SparkMaxConfig ayarlar = new SparkMaxConfig();
        
        // Atıcı motorları çok yüksek devirlerde döner, bu yüzden akım sınırı ekliyoruz. Şaseye göre biraz daha farklı ayarlanabilir (Bunu Constants'tan çekeceğiz).
        ayarlar.smartCurrentLimit(SHOOTER_MOTOR_CURRENT_LIMIT);
        
        // Atıcı motorlarında "Coast" modu kullanılır. Eğer Brake yaparsak, yüksek hızda dönen tekerlek bir anda durmaya çalışır ve kendini veya kayışları parçalayabilir. Coast diyerek yavaşça durmasını sağlıyoruz.
        ayarlar.idleMode(IdleMode.kCoast);

        // 4. TAKİPÇİ MOTORU AYARLAMA
        // Takipçi motor her zaman ana motorun yaptığı dönüşü taklit etsin.
        ayarlar.follow(aticiAnaMotor);
        aticiTakipciMotor.configure(ayarlar, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        // 5. ANA MOTORU AYARLAMA
        ayarlar.disableFollowerMode();
        // Eğer atıcının tasarımı gereği motorların ters dönmesi gerekiyorsa burayı true yaparız. Şimdilik false olarak bırakıyoruz.
        ayarlar.inverted(false);
        aticiAnaMotor.configure(ayarlar, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // 6. ATIŞ YAPMA METODU
    /**
     * Atıcı tekerleklerini istenilen hızda döndürür.
     * @param hiz Motor hızı (-1.0 ile 1.0 arası). Genelde atış için 1.0 (tam güç) kullanılır.
     */
    public void atisYap(double hiz) {
        // Sadece ana motora hız veriyoruz, takipçi motor zaten onu kopyalıyor.
        aticiAnaMotor.set(hiz);
    }

    // 7. MOTORLARI DURDURMA METODU
    // Atıcı motorlarını tamamen durdurur.
    public void durdur() {
        // Hızı 0.0 yaparak motorları durduruyoruz. IdleMode.kCoast seçtiğimiz için motorlar yavaşlayıp duracaktır.
        aticiAnaMotor.set(0.0);
    }

    @Override
    public void periodic() {
        // Atıcının hızını okumak veya ekrana yazdırmak istersek burayı kullanacağız.
    }
}