import cv2
import numpy as np

# ==========================================
# CONFIGURAÇÕES DO PADRÃO XADREZ
# ==========================================
# Número de INTERSEÇÕES INTERNAS do xadrez (Colunas, Linhas)
CHESSBOARD_SIZE = (10, 7) 

# Critérios de parada para o refinamento de cantos (subpixel)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepara os pontos 3D reais do xadrez (0,0,0), (1,0,0), (2,0,0) ...
# O tamanho do quadrado físico não importa muito para obter K e D, 
# mas criamos uma grade genérica baseada no número de interseções.
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)

# Arrays para armazenar os pontos 3D (mundo real) e 2D (imagem) de todas as capturas
objpoints = [] # Pontos 3D no espaço real
imgpoints = [] # Pontos 2D no plano da imagem

def main():
    cap = cv2.VideoCapture(1)
    
    # Resolução da camera
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

    print("======================================================")
    print("CALIBRAÇÃO DA CÂMERA")
    print("Mova o papel xadrez na frente da câmera.")
    print("Pressione 'c' para CAPTURAR uma pose (faça umas 15-20).")
    print("Pressione 'q' para CALCULAR e SALVAR a calibração.")
    print("======================================================")

    captures = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Erro ao acessar a câmera.")
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Encontra os cantos do xadrez
        ret_chess, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

        # Copia o frame para desenhar por cima sem afetar a imagem original
        display_frame = frame.copy()

        if ret_chess:
            # Refina a precisão dos cantos encontrados
            corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # Desenha os cantos coloridos na tela para você ver que funcionou
            cv2.drawChessboardCorners(display_frame, CHESSBOARD_SIZE, corners_refined, ret_chess)

        # Mostra o status na tela
        cv2.putText(display_frame, f"Capturas: {captures}/15+", (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow('Calibracao', display_frame)

        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('c') and ret_chess:
            # Salva os pontos da imagem atual
            objpoints.append(objp)
            imgpoints.append(corners_refined)
            captures += 1
            print(f"[{captures}] Pose capturada com sucesso!")
            
        elif key == ord('q'):
            break

    # Após apertar 'q', faz os cálculos se tivermos imagens suficientes
    if captures > 0:
        print("\nCalculando os parâmetros da câmera... Isso pode levar alguns segundos.")
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            objpoints, imgpoints, gray.shape[::-1], None, None
        )

        if ret:
            print("\nCalibração concluída com sucesso!")
            print("Matriz da Câmera (K):\n", camera_matrix)
            print("Coeficientes de Distorção (D):\n", dist_coeffs)

            # Salva os arquivos no mesmo diretório do script
            np.save('camera_matrix.npy', camera_matrix)
            np.save('dist_coeffs.npy', dist_coeffs)
            print("\nArquivos 'camera_matrix.npy' e 'dist_coeffs.npy' foram salvos!")
            print("Agora você pode rodar o script da Etapa 2.")
        else:
            print("\nFalha ao calibrar a câmera. Tente tirar mais fotos de ângulos diferentes.")
    else:
        print("\nNenhuma imagem foi capturada. Calibração cancelada.")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()