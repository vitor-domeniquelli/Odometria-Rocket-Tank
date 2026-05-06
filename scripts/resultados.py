import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_spatial_comparison(robot_file, gt_file):
    try:
        # 1. Carregamento dos dados do Robô
        # Mapeamento das 7 colunas conforme o SerialBT.printf do seu ESP32
        cols_robot = ['Tempo', 'X_imu', 'Y_imu', 'T_imu', 'Tck_E', 'Tck_D', 'Gz_raw']
        
        # Lemos o CSV ignorando o cabeçalho de texto enviado pelo resetOdometria()
        df_robot = pd.read_csv(robot_file, names=cols_robot, header=0, on_bad_lines='skip')
        
        # Conversão para numérico para garantir que não haja erros de string
        for col in df_robot.columns:
            df_robot[col] = pd.to_numeric(df_robot[col], errors='coerce')
        df_robot = df_robot.dropna()

        # 2. Carregamento do Ground Truth
        # O cabeçalho esperado é ['timestamp', 'x', 'y', 'theta'] conforme o script de captura ajustado
        df_gt = pd.read_csv(gt_file)

        # 3. Criação do Gráfico de Trajetória
        plt.figure(figsize=(10, 10))
        
        # Trajetória Real (Ground Truth) em preto[cite: 1, 2]
        plt.plot(df_gt['x'], df_gt['y'], 'k-', label='Ground Truth (Real)', linewidth=3, zorder=1)
        
        # Trajetória do Robô (Encoder + IMU) em azul[cite: 2]
        plt.plot(df_robot['X_imu'], df_robot['Y_imu'], 'b-', label='Estimativa Robô (IMU)', linewidth=2)

        # Marcações de Início (0,0)[cite: 2]
        plt.scatter(0, 0, color='green', s=200, label='Ponto de Partida', edgecolors='black', zorder=5)

        # 4. Identificação dos Pontos Finais[cite: 2]
        final_robot = df_robot.iloc[-1]
        final_gt = df_gt.iloc[-1]

        plt.scatter(final_robot['X_imu'], final_robot['Y_imu'], color='blue', marker='X', s=100, label='Fim (Robô)')
        plt.scatter(final_gt['x'], final_gt['y'], color='black', marker='X', s=100, label='Fim (Real)')

        plt.title('Comparação Espacial de Trajetória: Robô vs Ground Truth')
        plt.xlabel('X (metros)')
        plt.ylabel('Y (metros)')
        plt.axis('equal')
        plt.grid(True, linestyle=':', alpha=0.7)
        plt.legend()

        # 5. Cálculo do Erro de Pose Final[cite: 2]
        err_dist = np.sqrt((final_robot['X_imu'] - final_gt['x'])**2 + (final_robot['Y_imu'] - final_gt['y'])**2)
        err_theta = np.degrees(final_robot['T_imu'] - final_gt['theta'])

        print("-" * 40)
        print("RELATÓRIO DE ERRO NO MARCO FINAL")
        print("-" * 40)
        print(f"Erro de Posição Final:       {err_dist:.4f} m")
        print(f"Orientação Robô Final:       {np.degrees(final_robot['T_imu']):.2f}°")
        print(f"Orientação Real Final:       {np.degrees(final_gt['theta']):.2f}°")
        print(f"Erro Angular Final:          {err_theta:.2f}°")
        print("-" * 40)

        plt.show()

    except Exception as e:
        print(f"Erro ao processar dados: {e}")

# Execução (Certifique-se que os arquivos estão na mesma pasta)
plot_spatial_comparison('odometry_square.csv', 'ground_truth.csv')