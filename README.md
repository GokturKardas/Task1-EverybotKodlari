#EverybotKodlari

Everybot kodunu tekrardan yazıyoruz.

Comend-Based Framework

Everybotun bileşenleri:
-Drivetrain
-Intake
-Shooter
-Climber

Kod NEO motora göre yazılmıştır(drivetrain).

Drivetrain: Robotun şase yani alt gövde hareket bölümüdür. Biz bu dosyada drivetrain kodunu tank şaseye göre yazdık. Kodda şasemizde 4 adet motor olduğunu hayal ettik ve bu motorlardan sağdakini sağAnaMotor soldakini de solAnaMotor olmak üzere iki tanesini sürüş sistemine verdik. Çünkü kodumuzda Follower(takipçi) modunu kullandık. Bu moda göre sol arka motor sol ön motoru, sağ arka motor sağ ön motoru birebir taklit ediyor. Buna ek olarak kodumuzda Fren(kBrake) modunu kullandık(Idlemode.kBrake). Bu mod sayesinde sürücü kontrolcüden elini çektiği anda robot direkt olarak motorlarını durdurarak duruyor. Aynı zamanda şasede sağ ve sol motorlar birbirine zıt olarak montajlandığı için sol tarafı kodda ters çevirerek sürüşün doğru olmasını sağladık. 

Shooter: Robotun atıcı mekanizmasını yöneten subsytemdir. Bu kodda objelerin daha hızlı bir şekilde fırlatılabilmesi için motorları tam güç olarak çalıştırmaya karar verdik. Bunun için atisYap adında bir metod oluşturduk ve tam güç (1.0) verdik. Ayrıca bunda drivetrainin tersine Kaydırma(kCoast) kullandık çünkü motorlar tam güç çalıştığından dolayı duracağı zaman aniden dursaydı kayışlar veya robotun farklı mekanizmaları hasar görebilirdi. Bu mod sayesinde motorlar aniden değil yavaşlayarak durur ve güvenli bir duruş sağlar.