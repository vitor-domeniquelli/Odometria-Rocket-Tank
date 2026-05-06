# Rocket Tank: Odometria Híbrida e Ground Truth Visual

Projeto desenvolvido para a disciplina de **Robôs Móveis Autônomos (RMA)** na **UFABC**. O objetivo é comparar a localização estimada por sensores internos (encoders e IMU) com a posição real obtida por visão computacional.

## 📝 Descrição
O projeto utiliza um robô de esteiras para executar trajetórias lineares e quadradas. A precisão do deslocamento é validada através de um sistema de *Ground Truth* baseado em marcadores **ArUco**.

## 🛠️ Hardware e Tecnologias
- **Plataforma:** Chassi Rocket Tank (esteiras).
- **Controlador:** ESP32.
- **Sensores:** Encoders ópticos e IMU MPU6050.
- **Visão:** OpenCV com marcadores ArUco.

## 📂 Estrutura de Pastas
- `/firmware`: Código C++ do ESP32.
- `/scripts`: Scripts Python (ArUco tracking e plotagem).
- `/docs`: Relatório técnico e imagens.

## 👥 Integrantes
- Everton Zhu
- Fabrizio Rondon Pinheiro
- Guilherme Costacurta Bissoli
- Vitor Domeniquelli Chagas

**Orientação:** Profª Elvira Rafikova
