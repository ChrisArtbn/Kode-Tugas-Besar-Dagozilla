#include "MPU6500_Raw.h"
#include <Wire.h>
#include <math.h>

// ---  Definisi Pin  ---
#define PWM_L_1_PIN 14
#define PWM_R_1_PIN 27
#define PWM_L_2_PIN 17
#define PWM_R_2_PIN 5
#define PWM_L_3_PIN 4
#define PWM_R_3_PIN 2
#define EN_M13_PIN 16
#define EN_M2_PIN 13
#define SDA_PIN 21
#define SCL_PIN 22
// pin kicker + dribbler

// --- Konfigurasi PWM LEDC ---
const int PWM_CHAN[] = {0, 1, 2, 3, 4, 5};
const int PWM_PINS[] = {PWM_L_1_PIN, PWM_R_1_PIN, PWM_L_2_PIN, PWM_R_2_PIN, PWM_L_3_PIN, PWM_R_3_PIN};
const int PWM_FREQ = 5000;
const int PWM_RESOLUTION = 8;

// --- Objek & Variabel Global ---
MPU6500 mpu;
volatile float heading = 0.0;
float residual_gyro_z_bias = 0.0;
const float ALPHA = 0.98;

// Deklarasi Task
void task_SensorReader(void *pvParameters);
void task_MotorController(void *pvParameters);

// --- Deklarasi Fungsi Helper untuk Motor ---
void setMotorSpeeds(float m1_speed, float m2_speed, float m3_speed);
void calculateAndSetOmniWheels(float target_speed_y, float target_speed_x, float rotation_speed);


void setup() {
    Serial.begin(115200);
    Wire.begin(SDA_PIN, SCL_PIN);
    delay(1000);

    Serial.println("--- Kontrol Berbasis Lapangan (Field-Centric) ---");

    // Inisialisasi & Kalibrasi Sensor (sama seperti sebelumnya)
    if (!mpu.setup(0x68)) {
        while (1) { Serial.println("Koneksi MPU gagal."); delay(5000); }
    }
    Serial.println("Kalibrasi dimulai...");
    mpu.calibrateAccelGyro();
    int num_samples = 500;
    float temp_bias_sum = 0.0;
    for (int i = 0; i < num_samples; i++) {
        mpu.update(); temp_bias_sum += mpu.getGyroZ(); delay(5);
    }
    residual_gyro_z_bias = temp_bias_sum / num_samples;
    Serial.print("Kalibrasi selesai. Sisa Bias Gyro Z: ");
    Serial.println(residual_gyro_z_bias);

    // Setup Pin-pin Motor
    pinMode(EN_M13_PIN, OUTPUT);
    pinMode(EN_M2_PIN, OUTPUT);
    digitalWrite(EN_M13_PIN, HIGH);
    digitalWrite(EN_M2_PIN, HIGH);

    // Setup semua channel PWM
    for (int i = 0; i < 6; i++) {
        ledcAttach(PWM_PINS[i], PWM_FREQ, PWM_RESOLUTION);
    }
    // Membuat dua task
    xTaskCreatePinnedToCore(task_SensorReader, "SensorTask", 4096, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(task_MotorController, "MotorTask", 4096, NULL, 1, NULL, 1);

    Serial.println("Task Sensor dan Motor telah dibuat dan berjalan.");
    vTaskDelete(NULL);
}

void loop() {
    // Biarkan kosong
}

/**
 * @brief Task untuk membaca sensor. TIDAK ADA PERUBAHAN DI SINI.
 */
void task_SensorReader(void *pvParameters) {
    unsigned long last_update_time = micros();
    float pitch = 0.0; float roll = 0.0;
    for (;;) {
        mpu.update();
        unsigned long current_time = micros();
        float dt = (current_time - last_update_time) / 1000000.0;
        last_update_time = current_time;
        float acc_x = mpu.getAccX(); float acc_y = mpu.getAccY(); float acc_z = mpu.getAccZ();
        float accel_pitch = atan2(acc_y, acc_z) * 180.0 / M_PI;
        float accel_roll = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 180.0 / M_PI;
        float gyro_x_dps = mpu.getGyroX(); float gyro_y_dps = mpu.getGyroY();
        float gyro_z_dps = mpu.getGyroZ() - residual_gyro_z_bias;
        pitch = ALPHA * (pitch + gyro_y_dps * dt) + (1.0 - ALPHA) * accel_pitch;
        roll  = ALPHA * (roll + gyro_x_dps * dt) + (1.0 - ALPHA) * accel_roll;
        heading += gyro_z_dps * dt;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Task untuk mengontrol motor dengan logika kinematika.
 */
void task_MotorController(void *pvParameters) {
    const int speed = 150; // Kecepatan target (0-255)

    for (;;) { // Loop tak terbatas
        // --- FASE MAJU (BERGERAK KE ARAH 0 DERAJAT) ---
        Serial.println("MotorTask: Bergerak MAJU (Field-Centric)");
        calculateAndSetOmniWheels(speed, 0, 0); // target_speed_y = speed, x = 0, rotasi = 0
        vTaskDelay(3000 / portTICK_PERIOD_MS); // Tahan selama 3 detik

        // --- FASE BERHENTI ---
        Serial.println("MotorTask: BERHENTI");
        calculateAndSetOmniWheels(0, 0, 0);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Tahan selama 2 detik

        // --- FASE MUNDUR (BERGERAK KE ARAH 180 DERAJAT) ---
        Serial.println("MotorTask: Bergerak MUNDUR (Field-Centric)");
        calculateAndSetOmniWheels(-speed, 0, 0); // target_speed_y = -speed
        vTaskDelay(3000 / portTICK_PERIOD_MS); // Tahan selama 3 detik

        // --- FASE BERHENTI ---
        Serial.println("MotorTask: BERHENTI");
        calculateAndSetOmniWheels(0, 0, 0);
        vTaskDelay(2000 / portTICK_PERIOD_MS); // Tahan selama 2 detik
    }
}

/**
 * @brief Fungsi ini adalah inti dari kinematika omniwheel.
 * Ia mengubah kecepatan target di sumbu Y (maju/mundur), X (kiri/kanan),
 * dan rotasi menjadi kecepatan individual untuk setiap motor.
 * @param target_speed_y Kecepatan maju/mundur yang diinginkan (-255 to 255)
 * @param target_speed_x Kecepatan kiri/kanan yang diinginkan (-255 to 255)
 * @param rotation_speed Kecepatan rotasi yang diinginkan (-255 to 255)
 */
void calculateAndSetOmniWheels(float target_speed_y, float target_speed_x, float rotation_speed) {
    // Konversi heading robot saat ini dari derajat ke radian untuk fungsi sin/cos
    float heading_rad = heading * M_PI / 180.0;
    
    // --- Transformasi Koordinat ---
    // Mengubah kecepatan target dari kerangka acuan LAPANGAN ke kerangka acuan ROBOT.
    // Ini adalah langkah kunci untuk kontrol field-centric.
    float robot_speed_x = target_speed_x * cos(heading_rad) + target_speed_y * sin(heading_rad);
    float robot_speed_y = -target_speed_x * sin(heading_rad) + target_speed_y * cos(heading_rad);
    
    // --- Rumus Inverse Kinematics untuk 3 Roda Omni (120 derajat) ---
    // Asumsi: M1 di 0°, M2 di 120°, M3 di 240° relatif terhadap depan robot.
    // Kecepatan motor adalah proyeksi dari vektor kecepatan robot ke arah masing-masing motor.
    float m1_speed = robot_speed_y; // cos(0)=1, sin(0)=0
    float m2_speed = -robot_speed_x * sin(120 * M_PI / 180.0) + robot_speed_y * cos(120 * M_PI / 180.0);
    float m3_speed = -robot_speed_x * sin(240 * M_PI / 180.0) + robot_speed_y * cos(240 * M_PI / 180.0);

    // Tambahkan komponen rotasi ke setiap motor
    m1_speed += rotation_speed;
    m2_speed += rotation_speed;
    m3_speed += rotation_speed;

    // Panggil fungsi helper untuk mengatur PWM motor
    setMotorSpeeds(m1_speed, m2_speed, m3_speed);
}

/**
 * @brief Fungsi helper untuk mengatur PWM L/R berdasarkan nilai kecepatan.
 * @param m_speed Nilai kecepatan (-255 hingga 255). Negatif berarti mundur.
 */
void setMotorSpeeds(float m1_speed, float m2_speed, float m3_speed) {
    // Motor 1
    if (m1_speed >= 0) {
        ledcWrite(PWM_PINS[0], m1_speed); // L1
        ledcWrite(PWM_PINS[1], 0);        // R1
    } else {
        ledcWrite(PWM_PINS[0], 0);
        ledcWrite(PWM_PINS[1], -m1_speed);
    }

    // Motor 2
    if (m2_speed >= 0) {
        ledcWrite(PWM_PINS[2], m2_speed); // L2
        ledcWrite(PWM_PINS[3], 0);        // R2
    } else {
        ledcWrite(PWM_PINS[2], 0);
        ledcWrite(PWM_PINS[3], -m2_speed);
    }

    // Motor 3
    if (m3_speed >= 0) {
        ledcWrite(PWM_PINS[4], m3_speed); // L3
        ledcWrite(PWM_PINS[5], 0);        // R3
    } else {
        ledcWrite(PWM_PINS[4], 0);
        ledcWrite(PWM_PINS[5], -m3_speed);
    }
}
