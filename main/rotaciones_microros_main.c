#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include <unistd.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include <uros_network_interfaces.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/vector3.h>

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
#include <rmw_microros/rmw_microros.h>
#endif

static const char *TAG = "ROTACIONES_MICROROS";

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define MICRO_ROS_APP_STACK      16000
#define MICRO_ROS_APP_TASK_PRIO  5

#define PUBLISH_PERIOD_MS 20
#define N_STEPS 1000

static rcl_publisher_t vector_publisher;
static rcl_publisher_t vector_microrotado_publisher;

static rcl_subscription_t vector_subscriber;
static rcl_subscription_t euler_subscriber;

static geometry_msgs__msg__Vector3 vector_original;
static geometry_msgs__msg__Vector3 euler_angles;
static geometry_msgs__msg__Vector3 vector_rotado;
static geometry_msgs__msg__Vector3 vector_original;

// Callback para recibir los datos del topico /vector
void vector_sub_callback(const void * msgin) {
    const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
    vector_original.x = msg->x;
    vector_original.y = msg->y;
    vector_original.z = msg->z;
}

// Callback para recibir los datos del topico /euler
void euler_sub_callback(const void * msgin) {
    const geometry_msgs__msg__Vector3 * msg = (const geometry_msgs__msg__Vector3 *)msgin;
    euler_angles.x = msg->x; // roll
    euler_angles.y = msg->y; // pitch
    euler_angles.z = msg->z; // yaw
}

// TODO : Implementar una funcion que reciba los datos de los topicos /vector y /euler y realice la rotacion del vector original, para luego publicar el vector rotado en el topico /vector_rotado usando rotaciones.

static void rotar_vector(
    geometry_msgs__msg__Vector3 *vector, 
    geometry_msgs__msg__Vector3 *euler,
    geometry_msgs__msg__Vector3 *vector_rotado) 
{
    float roll = euler->x; 
    float pitch = euler->y;
    float yaw = euler->z;

    float cx = cos(roll);
    float sx = sin(roll);
    float cy = cos(pitch);
    float sy = sin(pitch);
    float cz = cos(yaw);
    float sz = sin(yaw);

    // Rz * Ry * Rx * v
    vector_rotado->x = vector->x*(cy*cz) + vector->y*(sx*sy*cz - cx*sz) + vector->z*(cx*sy*cz + sx*sz);
    vector_rotado->y = vector->x*(cy*sz) + vector->y*(sx*sy*sz + cx*cz) + vector->z*(cx*sy*sz - sx*cz);
    vector_rotado->z = vector->x*(-sy) + vector->y*(sx*cy) + vector->z*(cx*cy);
}

// TODO : Implementar una funcion que reciba los datos de los topicos /vector y /euler y realice la rotacion del vector original, para luego publicar el vector rotado en el topico /vector_microrotado usando microrotaciones.

static void microrotar_vector(
    geometry_msgs__msg__Vector3 *vector, 
    geometry_msgs__msg__Vector3 *euler, 
    geometry_msgs__msg__Vector3 *vector_rotado) 
{
    vector_rotado->x = vector->x;
    vector_rotado->y = vector->y;
    vector_rotado->z = vector->z;

    // 2. Calcular el diferencial para cada eje
    float roll = euler->x / N_STEPS;
    float pitch = euler->y / N_STEPS;
    float yaw = euler->z / N_STEPS;

    // 3. Microrotaciones sobre X (roll)
    for (int i = 0; i < N_STEPS; i++) {
        calcular_micro_rotacion(vector_rotado, 0, 0, roll);
    }

    // 4. Microrotaciones sobre Y (pitch)
    for (int i = 0; i < N_STEPS; i++) {
        calcular_micro_rotacion(vector_rotado, 0, pitch, 0);
    }

    // 5. Microrotaciones sobre Z (yaw)
    for (int i = 0; i < N_STEPS; i++) {
        calcular_micro_rotacion(vector_rotado, yaw, 0, 0);
    }
}

static void calcular_micro_rotacion(
    geometry_msgs__msg__Vector3 *vector, 
    float micro_yaw, float micro_pitch, float micro_roll) 
{
    float x_new = vector->x - micro_yaw * vector->y + micro_pitch * vector->z;
    float y_new = micro_yaw * vector->x + vector->y - micro_roll * vector->z;
    float z_new = -micro_pitch * vector->x + micro_roll * vector->y + vector->z;
    
    vector->x = x_new;
    vector->y = y_new;
    vector->z = z_new;
}

// TODO : Implementar una funcion que muestre el valor del vector original, el vector rotado y el vector microrotado en la terminal, para poder verificar que la rotacion se esta realizando correctamente.

static void timer_callback(rcl_timer_t *timer, int64_t last_call_time){
    
    (void) last_call_time;
    if (timer == NULL) {
        return;
    }
    
    rotar_vector(&vector_original, &euler_angles, &vector_rotado);
    microrotar_vector(&vector_original, &euler_angles, &vector_microrotado);
    
    RCSOFTCHECK(rcl_publish(&vector_publisher, &vector_rotado, NULL));
    RCSOFTCHECK(rcl_publish(&vector_microrotado_publisher, &vector_microrotado, NULL));
    
    ESP_LOGI(TAG, "Orig:(%.2f,%.2f,%.2f) | Rot:(%.2f,%.2f,%.2f) | uRot:(%.2f,%.2f,%.2f)", 
             vector_original.x, vector_original.y, vector_original.z,
             vector_rotado.x, vector_rotado.y, vector_rotado.z,
             vector_microrotado.x, vector_microrotado.y, vector_microrotado.z);
}

// TODO : Configurar el publicador del topico /vector_rotado y /vector_microrotado, para publicar los datos del vector rotado y el vector microrotado respectivamente.
// TODO : Configurar los subsriptores de los topico /vector y /euler
static void micro_ros_task(void *arg){
    while (1) {
        // 1. ESPERA / PING AL AGENTE
        ESP_LOGI(TAG, "Verificando Agente en IP: %s | Puerto: %s", CONFIG_MICRO_ROS_AGENT_IP, CONFIG_MICRO_ROS_AGENT_PORT);
        

        rcl_allocator_t allocator = rcl_get_default_allocator();
        rclc_support_t support;

        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        RCCHECK(rcl_init_options_init(&init_options, allocator));

#ifdef CONFIG_MICRO_ROS_ESP_XRCE_DDS_MIDDLEWARE
        rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
        RCCHECK(rmw_uros_options_set_udp_address(CONFIG_MICRO_ROS_AGENT_IP,
                                             CONFIG_MICRO_ROS_AGENT_PORT,
                                             rmw_options));
                                             
#endif

        // Intentar conectar con el agente directamente vía support_init
        rcl_ret_t rc = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

        if (rc != RCL_RET_OK) {
            ESP_LOGW(TAG, "No se pudo conectar con el Agente (error %d). Reintentando en 2s...", (int)rc);
            rcl_init_options_fini(&init_options);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue; // Reintentar en el ciclo while
        }

        ESP_LOGI(TAG, "¡Conectado exitosamente al Agente!");

        rcl_node_t node = rcl_get_zero_initialized_node();
        RCCHECK(rclc_node_init_default(&node, "potenciometro_node", "", &support));
        ESP_LOGI(TAG, "Nodo creado correctamente");

        // Publicadores de topicos /vector_rotado y /vector_microrotado
        RCCHECK(rclc_publisher_init_default(
            &vector_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "vector_rotado"));

        RCCHECK(rclc_publisher_init_default(
            &vector_microrotado_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "vector_microrotado"));

        // Suscriptores de topicos /vector y /euler
        RCCHECK(rclc_subscription_init_default(
            &vector_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "vector"));

        RCCHECK(rclc_subscription_init_default(
            &euler_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3),
            "euler"));

        rcl_timer_t timer = rcl_get_zero_initialized_timer();
        RCCHECK(rclc_timer_init_default2(
            &timer, &support, RCL_MS_TO_NS(PUBLISH_PERIOD_MS), timer_callback, true));

        rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
        RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
        RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(1)));

        // Bucle de publicación
        while (1) {
            rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Limpieza si sale del bucle
        rcl_publisher_fini(&vector_publisher, &node);
        rcl_publisher_fini(&vector_microrotado_publisher, &node);
        rcl_subscription_fini(&vector_subscriber, &node);
        rcl_subscription_fini(&euler_subscriber, &node);
        rcl_timer_fini(&timer);
        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
#if defined(CONFIG_MICRO_ROS_ESP_NETIF_WLAN) || defined(CONFIG_MICRO_ROS_ESP_NETIF_ENET)
    ESP_ERROR_CHECK(uros_network_interface_initialize());
#endif
    
    // DESACTIVAR POWER SAVE DE WI-FI
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_LOGI(TAG, "Wi-Fi Power Save desactivado (WIFI_PS_NONE)");

    xTaskCreate(micro_ros_task, "micro_ros_task",
                MICRO_ROS_APP_STACK, NULL, MICRO_ROS_APP_TASK_PRIO, NULL);
}