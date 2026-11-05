#!/usr/bin/env python3
"""
imu_noise_calibration_node.py  (ROS2 / Lyrical)

Node avulso (nao entra no pipeline do lio_sam) pra estimar via Allan
Variance os 4 parametros de ruido que o lio_sam.yaml pede:
  imuAccNoise, imuGyrNoise   -- ruido branco (N)
  imuAccBiasN, imuGyrBiasN   -- bias random walk (K)
De brinde, tambem mede a gravidade local (media de |accel| no periodo
parado), igual o campo "imuGravity" que ja existe comentado no seu yaml.

Como usar:
  1. Deixa o sensor PARADO numa superficie estavel (bancada, chao).
  2. chmod +x imu_noise_calibration_node.py
  3. ros2 run carmen_ros_bridge imu_noise_calibration_node.py --ros-args -p duration:=1800.0
     (ou solta com duration:=0.0 e aperta Ctrl+C quando quiser fechar --
      calcula com o que tiver coletado ate ali)
  4. Ele imprime o bloco pronto pra colar no lio_sam.yaml.

Params:
  imu_topic (string, default "/imu_raw")
  duration  (double, default 1800.0) -- segundos de coleta antes de calcular
                                          sozinho (0 = so calcula no Ctrl+C)

Nota sobre duracao: o ruido branco (N) sai estavel mesmo com poucos minutos.
O bias random walk (K) so fica confiavel com bastante dado parado -- testei
com simulacao e com <10min o erro passa de 50%; a partir de ~30min-1h ja fica
razoavel, 1-2h fica bom. Se voce fechar cedo (Ctrl+C), o node avisa que o K
saiu de um log curto, mas calcula do mesmo jeito -- util pra iterar rapido e
refinar depois com uma coleta mais longa.
"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
import numpy as np
from sensor_msgs.msg import Imu


class ImuNoiseCalibrationNode(Node):
    def __init__(self):
        super().__init__("imu_noise_calibration_node")
        self.imu_topic = self.declare_parameter('imu_topic', '/imu_raw').value
        self.duration = self.declare_parameter('duration', 1800.0).value

        self.t = []
        self.ax, self.ay, self.az = [], [], []
        self.gx, self.gy, self.gz = [], [], []

        self.t0 = None
        self._finished = False

        self.sub = self.create_subscription(Imu, self.imu_topic, self._cb, 2000)

        self._log("info", "[imu_noise_calib] ouvindo %s -- deixa o sensor PARADO.",
                       self.imu_topic)
        self._log("info", "[imu_noise_calib] Ctrl+C a qualquer momento calcula com o "
                       "que ja foi coletado.")
        if self.duration > 0:
            self._log("info", "[imu_noise_calib] calcula sozinho em %.0fs (~%.1f min) "
                           "se nao interromper antes.",
                           self.duration, self.duration / 60.0)

    def _log(self, level, message, *args):
        text = message % args if args else message
        if level == 'info':
            self.get_logger().info(text)
        elif level == 'warning':
            self.get_logger().warning(text)
        else:
            self.get_logger().error(text)

    def _cb(self, msg):
        now = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if now <= 0.0:
            now = self.get_clock().now().nanoseconds * 1e-9
        if self.t0 is None:
            self.t0 = now
        self.t.append(now - self.t0)
        self.ax.append(msg.linear_acceleration.x)
        self.ay.append(msg.linear_acceleration.y)
        self.az.append(msg.linear_acceleration.z)
        self.gx.append(msg.angular_velocity.x)
        self.gy.append(msg.angular_velocity.y)
        self.gz.append(msg.angular_velocity.z)

        if self.duration > 0 and (now - self.t0) >= self.duration:
            self._on_shutdown()

    # ─── Allan variance (overlapping), padrao IEEE-STD-952 ────────────────
    @staticmethod
    def _allan_variance(data, dt):
        """
        data: array 1D de amostras cruas (accel ou giro, mesma unidade)
        dt:   periodo de amostragem (s)
        Retorna (taus, allan_dev) na mesma unidade de 'data'.
        """
        n = len(data)
        max_m = max(n // 2, 1)
        ms = np.unique(np.logspace(0, np.log10(max_m), 100).astype(int))
        ms = ms[ms > 0]

        theta = np.cumsum(data) * dt  # integral (velocidade ou angulo)
        taus, avar = [], []
        for m in ms:
            if 2 * m >= n:
                continue
            tau = m * dt
            num = theta[2 * m:n] - 2.0 * theta[m:n - m] + theta[0:n - 2 * m]
            av = np.mean(num ** 2) / (2.0 * tau ** 2)
            taus.append(tau)
            avar.append(av)
        return np.array(taus), np.sqrt(np.array(avar))

    @staticmethod
    def _fit_noise_params(taus, adev):
        """
        Le as duas assintotas classicas do Allan deviation em log-log:
          slope -1/2 -> N (ruido branco / "noise density")
          slope +1/2 -> K (bias random walk)
        Convencao IEEE-STD-952: N lido em tau=1, K lido em tau=3.
        """
        log_tau = np.log10(taus)
        log_adev = np.log10(adev)
        slopes = np.gradient(log_adev, log_tau)

        idx_n = int(np.argmin(np.abs(slopes - (-0.5))))
        idx_k = int(np.argmin(np.abs(slopes - 0.5)))

        n_val = adev[idx_n] * (1.0 / taus[idx_n]) ** (-0.5)
        k_val = adev[idx_k] * (3.0 / taus[idx_k]) ** 0.5
        return float(n_val), float(k_val)

    def _analyze(self):
        n = len(self.t)
        if n < 50:
            self._log("error", "[imu_noise_calib] so %d amostras -- deixa coletar mais "
                          "antes de fechar. Nada foi calculado.", n)
            return

        dt = float(np.mean(np.diff(self.t)))
        fs = 1.0 / dt
        elapsed = self.t[-1]
        self._log("info", "[imu_noise_calib] %d amostras, %.1fs coletados, fs~=%.1fHz",
                       n, elapsed, fs)
        if elapsed < 1800.0:
            self._log("warning", "[imu_noise_calib] menos de 30min parado -- o ruido "
                           "branco (N) sai OK, mas o bias random walk (K) pode "
                           "errar bastante (testado: >50% de erro com <10min). "
                           "Ideal: 1-2h pra K confiavel.")

        results = {}
        for name, sig in (('accX', self.ax), ('accY', self.ay), ('accZ', self.az),
                           ('gyrX', self.gx), ('gyrY', self.gy), ('gyrZ', self.gz)):
            data = np.array(sig)
            taus, adev = self._allan_variance(data, dt)
            if len(taus) < 5:
                self._log("warning", "[imu_noise_calib] %s: poucos clusters de Allan "
                               "variance, pulando (coleta mais tempo)", name)
                continue
            results[name] = self._fit_noise_params(taus, adev)

        if not results:
            self._log("error", "[imu_noise_calib] nao deu pra calcular nada -- "
                          "coleta mais dado.")
            return

        def avg(keys):
            vals = [results[k] for k in keys if k in results]
            return None if not vals else vals

        acc_keys = [k for k in ('accX', 'accY', 'accZ') if k in results]
        gyr_keys = [k for k in ('gyrX', 'gyrY', 'gyrZ') if k in results]
        acc_n = np.mean([results[k][0] for k in acc_keys]) if acc_keys else float('nan')
        gyr_n = np.mean([results[k][0] for k in gyr_keys]) if gyr_keys else float('nan')
        acc_k = np.mean([results[k][1] for k in acc_keys]) if acc_keys else float('nan')
        gyr_k = np.mean([results[k][1] for k in gyr_keys]) if gyr_keys else float('nan')

        # gravidade local -- ja temos os dados de accel na mao mesmo
        g_mag = float(np.mean(np.sqrt(np.array(self.ax) ** 2 +
                                       np.array(self.ay) ** 2 +
                                       np.array(self.az) ** 2)))

        self._log("info", "=" * 70)
        self._log("info", "Resultado por eixo (N=ruido branco, K=bias random walk):")
        for name, (n_val, k_val) in results.items():
            self._log("info", "  %-5s  N=%.6e   K=%.6e", name, n_val, k_val)
        self._log("info", "=" * 70)
        self._log("info", 
            "Cole no lio_sam.yaml:\n"
            "  imuAccNoise: %.6f\n"
            "  imuGyrNoise: %.6f\n"
            "  imuAccBiasN: %.6f\n"
            "  imuGyrBiasN: %.6f\n"
            "  imuGravity: %.6f   # medido agora, %d amostras, %.1fs parado\n",
            acc_n, gyr_n, acc_k, gyr_k, g_mag, n, elapsed)
        self._log("info", "=" * 70)

    def _on_shutdown(self):
        if self._finished:
            return
        self._finished = True
        self._analyze()


if __name__ == '__main__':
    rclpy.init()
    node = ImuNoiseCalibrationNode()
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node._on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
