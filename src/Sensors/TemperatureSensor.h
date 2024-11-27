#include <DHT.h>

class TemperatureSensor {
    public:
        struct SensorData {
            float humidity;
            float temperature;
            float heatIndex;
        };

        enum SensorType { DHT_11 = DHT11,
                        DHT_22 = DHT22 };

        static SensorData readData(SensorType sensorType, const int gpioPin);
};
