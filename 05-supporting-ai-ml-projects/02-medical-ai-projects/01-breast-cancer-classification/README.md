# 🧠 Breast Cancer Classification using FCNN

This project focuses on classifying breast tumors as **malignant** or **benign** using a **Fully Connected Neural Network (FCNN)** on the **Wisconsin Diagnostic Breast Cancer (WDBC)** dataset.

## 📌 Motivation
Early and accurate diagnosis of breast cancer can save lives. This project explores the use of deep learning for binary classification based on structured data.

## 📊 Dataset
- **Source**: UCI Machine Learning Repository  
- **Samples**: 569  
- **Features**: 30 numeric  
- **Label**: Malignant (1), Benign (0)

## 🧪 Preprocessing
- Dropped ID column
- Label encoded diagnosis
- Normalized features
- Split into train/test

## 🧠 Model
- Fully Connected Neural Network (FCNN)
- ReLU + Sigmoid activations
- Dropout Regularization
- Binary Crossentropy loss
- Adam optimizer

## 📈 Evaluation
- Logistic Regression: 97.66%
- FCNN: 96.72%
- Random Forest: 95.78%
- Gradient Boosting: 95.08%

## ▶️ Run the Project

### 🔹 Option 1: Google Colab (Recommended for Beginners)

Click the button below to open the notebook in **Google Colab** and run it in the cloud — no installation needed.

[![Open In Colab](https://colab.research.google.com/assets/colab-badge.svg)](https://colab.research.google.com/github/Jrobo/RoboticsProjectsLab-computer-vision-projects-segmentation-breast-cancer-classification/blob/main/BreastCancerClassification_FCNN.ipynb)

---

### 🔹 Option 2: Run Locally

1. Clone the repo:
```bash
git clone https://github.com/Jrobo/RoboticsProjectsLab-computer-vision-projects-segmentation-breast-cancer-classification.git
cd RoboticsProjectsLab-computer-vision-projects-segmentation-breast-cancer-classification

