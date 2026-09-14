#include "Common_used.h"
#include "trace_tune.h"
#include "pid.h"
#include "Trace_base.h"

/* Runtime defaults. */
float g_tune_angle_kp = ANGLE_KP;
float g_tune_angle_ki = ANGLE_KI;
float g_tune_angle_kd = ANGLE_KD;
float g_tune_pos_kp = POS_KP;
float g_tune_pos_ki = POS_KI;
float g_tune_pos_kd = POS_KD;
float g_tune_gray_kp = GRAY_KP;
float g_tune_gray_ki = GRAY_KI;
float g_tune_gray_kd = GRAY_KD;
float g_tune_speed = TRACE_BASE_SPEED;
float g_tune_wmax = TRACE_W_MAX;
/* Keep the original line-follow behavior until bias is explicitly set. */
float g_tune_pos_bias = 0.0f;
volatile uint8_t g_tune_control_override = 0U;

volatile uint8_t g_tune_monitor = 0U;
volatile uint16_t g_tune_monitor_rate = 10U;

float g_tune_step_akp = 0.01f;
float g_tune_step_aki = 0.001f;
float g_tune_step_akd = 0.001f;
float g_tune_step_pkp = 0.01f;
float g_tune_step_pki = 0.001f;
float g_tune_step_pkd = 0.001f;
float g_tune_step_vmax = 0.05f;
float g_tune_step_wmax = 0.05f;
float g_tune_step_gkp = 0.01f;
float g_tune_step_gki = 0.001f;
float g_tune_step_gkd = 0.001f;

extern pid_type_def g_pid_angle;
extern pid_type_def g_pid_pos;
extern pid_type_def g_pid_gray;

#define TUNE_LINE_MAX 32U

static char tune_line[TUNE_LINE_MAX];
static volatile uint8_t tune_len = 0U;
static volatile uint8_t in_tune = 0U;
static volatile uint8_t tune_pending = 0U;
static uint16_t tune_monitor_divider = 0U;
static volatile uint8_t tune_snapshot_pending = 0U;

static float tune_parse_float(const char *s)
{
    float value = 0.0f;
    float fraction = 0.1f;
    uint8_t negative = 0U;
    uint8_t seen = 0U;

    if (*s == '-') {
        negative = 1U;
        s++;
    } else if (*s == '+') {
        s++;
    }

    while (*s >= '0' && *s <= '9') {
        value = value * 10.0f + (float)(*s - '0');
        s++;
        seen = 1U;
    }

    if (*s == '.') {
        s++;
        while (*s >= '0' && *s <= '9') {
            value += (float)(*s - '0') * fraction;
            fraction *= 0.1f;
            s++;
            seen = 1U;
        }
    }

    if (!seen) {
        return 0.0f;
    }
    return negative ? -value : value;
}

static float tune_adjust(float current, float value, int is_delta)
{
    float next = is_delta ? (current + value) : value;
    return (next < 0.0f) ? 0.0f : next;
}

static void tune_apply_gains(void)
{
    if (!g_tune_control_override) {
        return;
    }

    g_pid_angle.Kp = g_tune_angle_kp;
    g_pid_angle.Ki = g_tune_angle_ki;
    g_pid_angle.Kd = g_tune_angle_kd;

    g_pid_gray.Kp=g_tune_gray_kp;
    g_pid_gray.Kd=g_tune_gray_kd;
    g_pid_gray.Ki=g_tune_gray_ki;

    g_pid_pos.Kp = g_tune_pos_kp;
    g_pid_pos.Ki = g_tune_pos_ki;
    g_pid_pos.Kd = g_tune_pos_kd;
    g_pid_pos.max_out = g_tune_wmax;
}

static void tune_print_params(void)
{
    printf("# pid akp=%.4f aki=%.4f akd=%.4f pkp=%.4f pki=%.4f pkd=%.4f\r\n",
           (double)g_tune_angle_kp, (double)g_tune_angle_ki,
           (double)g_tune_angle_kd, (double)g_tune_pos_kp,
           (double)g_tune_pos_ki, (double)g_tune_pos_kd);
    printf("# gray kp=%.4f ki=%.4f kd=%.4f step=%.4f/%.4f/%.4f\r\n",
           (double)g_tune_gray_kp, (double)g_tune_gray_ki, (double)g_tune_gray_kd,
           (double)g_tune_step_gkp, (double)g_tune_step_gki, (double)g_tune_step_gkd);
    printf("# limit vmax=%.3f wmax=%.3f bias=%.3f mon=%u rate=%u\r\n",
           (double)g_tune_speed, (double)g_tune_wmax,
           (double)g_tune_pos_bias, (unsigned)g_tune_monitor,
           (unsigned)g_tune_monitor_rate);
    printf("# step akp=%.4f aki=%.4f akd=%.4f pkp=%.4f pki=%.4f pkd=%.4f vmax=%.3f wmax=%.3f\r\n",
           (double)g_tune_step_akp, (double)g_tune_step_aki,
           (double)g_tune_step_akd, (double)g_tune_step_pkp,
           (double)g_tune_step_pki, (double)g_tune_step_pkd,
           (double)g_tune_step_vmax, (double)g_tune_step_wmax);
}

static void tune_print_help(void)
{
    printf("# pid: #akp/#aki/#akd value, #pkp/#pki/#pkd value\r\n");
    printf("# gray: #gkp/#gki/#gkd value 或 #gkp+/#gkp-, 步进用 #step gkp X\r\n");
    printf("# limit: #vmax value, #wmax value, #bias -40\r\n");
    printf("# monitor: #mon 1|0, #rate 10, #snap, #get, #help\r\n");
    printf("# relative: #pkp +0.01, #step pkp 0.01, #pkp+/#pkp-\r\n");
}

static void tune_process_line(void)
{
    char *value_text = tune_line;
    char command[10];
    uint8_t command_len = 0U;
    float value;
    int is_delta;

    while (*value_text && *value_text != ' ' && *value_text != '\t') {
        if (command_len < sizeof(command) - 1U) {
            command[command_len++] = *value_text;
        }
        value_text++;
    }
    command[command_len] = '\0';
    while (*value_text == ' ' || *value_text == '\t') {
        value_text++;
    }

    is_delta = (*value_text == '+' || *value_text == '-');
    value = tune_parse_float(value_text);

    if (strcmp(command, "get") == 0) {
        tune_print_params();
        return;
    }
    if (strcmp(command, "help") == 0) {
        tune_print_help();
        return;
    }
    if (strcmp(command, "bias") == 0) {
        g_tune_pos_bias = value;
        g_tune_control_override = 1U;
        tune_print_params();
        return;
    }
    if (strcmp(command, "mon") == 0) {
        g_tune_monitor = (value != 0.0f) ? 1U : 0U;
        tune_monitor_divider = 0U;
        printf("# monitor=%u\r\n", (unsigned)g_tune_monitor);
        return;
    }
    if (strcmp(command, "rate") == 0) {
        uint32_t rate = (value < 1.0f) ? 1U : (uint32_t)value;
        if (rate > 1000U) {
            rate = 1000U;
        }
        g_tune_monitor_rate = (uint16_t)rate;
        tune_monitor_divider = 0U;
        printf("# rate=%u\r\n", (unsigned)g_tune_monitor_rate);
        return;
    }
    if (strcmp(command, "snap") == 0) {
        tune_snapshot_pending = 1U;
        return;
    }

    if (strcmp(command, "step") == 0) {
        char parameter[10];
        char *step_text = value_text;
        uint8_t parameter_len = 0U;

        while (*step_text && *step_text != ' ' && *step_text != '\t') {
            if (parameter_len < sizeof(parameter) - 1U) {
                parameter[parameter_len++] = *step_text;
            }
            step_text++;
        }
        parameter[parameter_len] = '\0';
        while (*step_text == ' ' || *step_text == '\t') {
            step_text++;
        }
        value = tune_parse_float(step_text);

        if (strcmp(parameter, "akp") == 0) g_tune_step_akp = value;
        else if (strcmp(parameter, "aki") == 0) g_tune_step_aki = value;
        else if (strcmp(parameter, "akd") == 0) g_tune_step_akd = value;
        else if (strcmp(parameter, "gkp") == 0) g_tune_step_gkp = value;
        else if (strcmp(parameter, "gki") == 0) g_tune_step_gki = value;
        else if (strcmp(parameter, "gkd") == 0) g_tune_step_gkd = value;
         else if (strcmp(parameter, "pkp") == 0) g_tune_step_pkp = value;
        else if (strcmp(parameter, "pki") == 0) g_tune_step_pki = value;
        else if (strcmp(parameter, "pkd") == 0) g_tune_step_pkd = value;
        else if (strcmp(parameter, "vmax") == 0) g_tune_step_vmax = value;
        else if (strcmp(parameter, "wmax") == 0) g_tune_step_wmax = value;
        else {
            printf("# unknown step parameter: %s\r\n", parameter);
            tune_print_help();
            return;
        }
        tune_print_params();
        return;
    }

    if (strcmp(command, "akp") == 0)
        g_tune_angle_kp = tune_adjust(g_tune_angle_kp, value, is_delta);
    else if (strcmp(command, "akp+") == 0)
        g_tune_angle_kp = tune_adjust(g_tune_angle_kp, g_tune_step_akp, 1);
    else if (strcmp(command, "akp-") == 0)
        g_tune_angle_kp = tune_adjust(g_tune_angle_kp, -g_tune_step_akp, 1);
    else if (strcmp(command, "aki") == 0)
        g_tune_angle_ki = tune_adjust(g_tune_angle_ki, value, is_delta);
    else if (strcmp(command, "aki+") == 0)
        g_tune_angle_ki = tune_adjust(g_tune_angle_ki, g_tune_step_aki, 1);
    else if (strcmp(command, "aki-") == 0)
        g_tune_angle_ki = tune_adjust(g_tune_angle_ki, -g_tune_step_aki, 1);
    else if (strcmp(command, "akd") == 0)
        g_tune_angle_kd = tune_adjust(g_tune_angle_kd, value, is_delta);
    else if (strcmp(command, "akd+") == 0)
        g_tune_angle_kd = tune_adjust(g_tune_angle_kd, g_tune_step_akd, 1);
    else if (strcmp(command, "akd-") == 0)
        g_tune_angle_kd = tune_adjust(g_tune_angle_kd, -g_tune_step_akd, 1);
    else if (strcmp(command, "gkp") == 0)
        g_tune_gray_kp = tune_adjust(g_tune_gray_kp, value, is_delta);
    else if (strcmp(command, "gkp+") == 0)
        g_tune_gray_kp = tune_adjust(g_tune_gray_kp, g_tune_step_gkp, 1);
    else if (strcmp(command, "gkp-") == 0)
        g_tune_gray_kp = tune_adjust(g_tune_gray_kp, -g_tune_step_gkp, 1);
    else if (strcmp(command, "gki") == 0)
        g_tune_gray_ki = tune_adjust(g_tune_gray_ki, value, is_delta);
    else if (strcmp(command, "gki+") == 0)
        g_tune_gray_ki = tune_adjust(g_tune_gray_ki, g_tune_step_gki, 1);
    else if (strcmp(command, "gki-") == 0)
        g_tune_gray_ki = tune_adjust(g_tune_gray_ki, -g_tune_step_gki, 1);
    else if (strcmp(command, "gkd") == 0)
        g_tune_gray_kd = tune_adjust(g_tune_gray_kd, value, is_delta);
    else if (strcmp(command, "gkd+") == 0)
        g_tune_gray_kd = tune_adjust(g_tune_gray_kd, g_tune_step_gkd, 1);
    else if (strcmp(command, "gkd-") == 0)
        g_tune_gray_kd = tune_adjust(g_tune_gray_kd, -g_tune_step_gkd, 1);
    else if (strcmp(command, "pkp") == 0)
        g_tune_pos_kp = tune_adjust(g_tune_pos_kp, value, is_delta);
    else if (strcmp(command, "pkp+") == 0)
        g_tune_pos_kp = tune_adjust(g_tune_pos_kp, g_tune_step_pkp, 1);
    else if (strcmp(command, "pkp-") == 0)
        g_tune_pos_kp = tune_adjust(g_tune_pos_kp, -g_tune_step_pkp, 1);
    else if (strcmp(command, "pki") == 0)
        g_tune_pos_ki = tune_adjust(g_tune_pos_ki, value, is_delta);
    else if (strcmp(command, "pki+") == 0)
        g_tune_pos_ki = tune_adjust(g_tune_pos_ki, g_tune_step_pki, 1);
    else if (strcmp(command, "pki-") == 0)
        g_tune_pos_ki = tune_adjust(g_tune_pos_ki, -g_tune_step_pki, 1);
    else if (strcmp(command, "pkd") == 0)
        g_tune_pos_kd = tune_adjust(g_tune_pos_kd, value, is_delta);
    else if (strcmp(command, "pkd+") == 0)
        g_tune_pos_kd = tune_adjust(g_tune_pos_kd, g_tune_step_pkd, 1);
    else if (strcmp(command, "pkd-") == 0)
        g_tune_pos_kd = tune_adjust(g_tune_pos_kd, -g_tune_step_pkd, 1);
    else if (strcmp(command, "vmax") == 0)
        g_tune_speed = tune_adjust(g_tune_speed, value, is_delta);
    else if (strcmp(command, "vmax+") == 0)
        g_tune_speed = tune_adjust(g_tune_speed, g_tune_step_vmax, 1);
    else if (strcmp(command, "vmax-") == 0)
        g_tune_speed = tune_adjust(g_tune_speed, -g_tune_step_vmax, 1);
    else if (strcmp(command, "wmax") == 0)
        g_tune_wmax = tune_adjust(g_tune_wmax, value, is_delta);
    else if (strcmp(command, "wmax+") == 0)
        g_tune_wmax = tune_adjust(g_tune_wmax, g_tune_step_wmax, 1);
    else if (strcmp(command, "wmax-") == 0)
        g_tune_wmax = tune_adjust(g_tune_wmax, -g_tune_step_wmax, 1);
    else {
        printf("# unknown command: %s\r\n", command);
        tune_print_help();
        return;
    }

    tune_apply_gains();
    g_tune_control_override = 1U;
    PID_clear(&g_pid_angle);
    PID_clear(&g_pid_pos);
    tune_print_params();
}

bool Trace_Tune_OnByte(uint8_t byte)
{
    if (!in_tune) {
        if (byte != '#') {
            return false;
        }
        in_tune = 1U;
        tune_len = 0U;
        return true;
    }

    if (byte == '\r' || byte == '\n') {
        tune_line[tune_len] = '\0';
        tune_pending = 1U;
        in_tune = 0U;
        tune_len = 0U;
        return true;
    }

    if (tune_len < TUNE_LINE_MAX - 1U) {
        tune_line[tune_len++] = (char)byte;
    }
    return true;
}

void Trace_Tune_Service(void)
{
    tune_apply_gains();
    if (tune_pending) {
        tune_pending = 0U;
        tune_process_line();
    }
}

void Trace_Tune_Record(float angle, float position, float target_position,
                       float v, float w)
{
    if (!g_tune_monitor && !tune_snapshot_pending) {
        return;
    }

    if (!tune_snapshot_pending) {
        tune_monitor_divider++;
        if (tune_monitor_divider < g_tune_monitor_rate) {
            return;
        }
        tune_monitor_divider = 0U;
    }
    tune_snapshot_pending = 0U;

    printf("TUNE,ang=%.2f,pos=%.2f,aerr=%.2f,perr=%.2f,target=%.2f,"
           "v=%.3f,w=%.3f,ap=%.3f,ai=%.3f,ad=%.3f,ao=%.3f,"
           "pp=%.3f,pi=%.3f,pd=%.3f,po=%.3f,bias=%.2f\r\n",
           (double)angle, (double)position,
           (double)g_pid_angle.error[0], (double)g_pid_pos.error[0],
           (double)target_position, (double)v, (double)w,
           (double)g_pid_angle.Pout, (double)g_pid_angle.Iout,
           (double)g_pid_angle.Dout, (double)g_pid_angle.out,
           (double)g_pid_pos.Pout, (double)g_pid_pos.Iout,
           (double)g_pid_pos.Dout, (double)g_pid_pos.out,
           (double)g_tune_pos_bias);
}
