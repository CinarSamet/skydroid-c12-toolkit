# skydroid-c12-toolkit (Türkçe Rehber)

[English README here](../README.md).

Bu paket iki şeyi yapar:

1. **Hareket**: Skydroid C12 gimbal kamerayı sağa/sola/yukarı/aşağı döndürür.
2. **IMU**: Kameranın gerçekten hangi açıda durduğunu (yaw/pitch/roll) okur.

Skydroid, C12 için resmi bir SDK yayınlamıyor. Bu proje, resmi Android FPV
uygulamasının kullandığı `#TP` UDP protokolünü **gerçek donanımda
doğrulanmış şekilde** tersine mühendislikle çözüp yeniden yazıyor.

## Kurulum

```bash
git clone https://github.com/CinarSamet/skydroid-c12-toolkit.git
cd skydroid-c12-toolkit
pip install .
```

Harici bağımlılık yoktur - sadece Python 3.8+ standart kütüphanesi.

Bilgisayarınızı kameranın alt ağına alın (C12 varsayılan olarak
`192.168.144.x` üzerinde, Ethernet ile çalışır):

```bash
sudo ip addr add 192.168.144.10/24 dev eth0
ping 192.168.144.108   # varsayılan kamera IP'si, sizinkinde farklı olabilir
```

## IMU nedir, ne işe yarar?

IMU, kameranın içindeki küçük bir sensör. "Ben şu an 20 derece yukarı
bakıyorum" gibi kameranın **gerçek** açısını ölçüp bize bildirir.

Kameraya "20 derece dön" komutu gönderdiğimizde, kameranın gerçekten tam
20 dereceye mi gittiğini, komutu gönderirken bilemeyiz. IMU sayesinde:

- Kameranın **gerçekten hedefe ulaştığını** teyit ederiz (tahmin değil).
- Zamanla oluşabilecek sapmayı (drift) düzeltiriz.
- Biri kamerayı elle oynatırsa bunu fark ederiz.

Paket, IMU verisi varsa bunu otomatik kullanır; IMU'dan veri gelmiyorsa
sorun değil, eski usul (süreye dayalı tahmin) ile devam eder.

## Terminalden kullanım

```bash
skydroid-c12 192.168.144.108 right 30      # SU ANKI konumdan 30 derece saga (goreli)
skydroid-c12 192.168.144.108 left 20       # su anki konumdan 20 derece sola (goreli)
skydroid-c12 192.168.144.108 tilt 10       # MUTLAK 10 derece yukari
skydroid-c12 192.168.144.108 tilt -10      # MUTLAK 10 derece asagi
skydroid-c12 192.168.144.108 goto 45       # MUTLAK 45 derece pan
skydroid-c12 192.168.144.108 point 30 10   # pan=30 VE tilt=10'a AYNI ANDA
skydroid-c12 192.168.144.108 center        # merkeze don
skydroid-c12 192.168.144.108 stop          # aninda durdur

skydroid-c12 192.168.144.108 imu           # TEK SEFERLIK IMU oku
skydroid-c12 192.168.144.108 watch 10      # 10 saniye SUREKLI IMU izle
```

Türkçe alias'lar da çalışır: `sag`, `sol`, `egim`, `nokta`, `merkez`,
`dur`, `izle`. Tüm seçenekler: `skydroid-c12 --help`.

`right`/`left` **şu anki konumunuza göre** göreli hareket eder. `goto` ve
`point` ise her zaman **merkeze göre mutlak** açıya gider.

## Kod içinden kullanım

```python
from skydroid_c12 import Camera

with Camera("192.168.144.108") as cam:
    cam.center()
    cam.goto_pan(30)              # mutlak, IMU ile teyitli (varsa)
    cam.goto_tilt(10)
    cam.goto(pan=-20, tilt=5)     # pan+tilt ayni anda

    cam.move_right(10)            # su anki konumdan itibaren goreli
    cam.move_tilt_down(5)

    sample = cam.get_attitude()   # ham yaw/pitch/roll (None olabilir)
    if sample:
        print(sample.yaw_deg, sample.pitch_deg, sample.roll_deg)

    print(cam.pan_deg, cam.tilt_deg)   # bilinen konum (IMU taze ise GERCEK)

    cam.stop_now()
```

Tek seferlik, bağlantı açık tutmadan IMU okumak isterseniz:

```python
from skydroid_c12 import get_attitude_once

sample = get_attitude_once("192.168.144.108")
print(sample.yaw_deg, sample.pitch_deg, sample.roll_deg)
```

## Doğrulanmış teknik bilgiler (gerçek C12 donanımında)

- Kontrol portu: **UDP 5000** (genel dokümantasyonun varsaydığı 9002
  DEĞİL - kendi cihazınızda farklı olabilir, doğrulamadan güvenmeyin).
- PTZ sabit komutları: `STOP=00 UP=01 DOWN=02 LEFT=03 RIGHT=04
  CENTER=05`.
- `GAY` (pan mutlak açı): ham işareti `GSY`'nin **tersi** - pozitif ham
  değer SOLA döndürür. SDK bunu kendi içinde düzeltip dışarıya tutarlı
  bir API sunuyor.
- `GAP` (tilt mutlak açı): işareti `GSP` ile aynı yönde, çevirme yok.
- `GAA`/`GAC`: attitude telemetrisi. Gerçek donanımda doğrulandı: `tilt
  20` komutunda `GAC.pitch` gerçek zamanlı olarak 0'dan 20.00 dereceye
  yükseldi.
- Tilt ekseninde ~45° civarında fiziksel/mekanik bir sınır gözlemlendi,
  yazılımda güvenlik sınırı olarak uygulanıyor.

Kendi firmware sürümünüzde davranış farklı olabilir - üretim/güvenlik
kritik bir kuruluma almadan önce küçük adımlarla dikkatlice test edin.

## Testler

```bash
pip install -e ".[dev]"
pytest -v
```

Testler tamamen sahte (mock) bir UDP kamera üzerinden çalışır - gerçek
donanıma ihtiyaç yoktur.
