# SmartBioGas-PIoT: Physics-Informed Edge AI and TinyML for Real-Time Monitoring and Methane Rate Prediction in Anaerobic Digesters

This repository presents the official implementation of SmartBioGas-PIoT, a hybrid artificial intelligence–driven renewable energy monitoring framework for real-time methane production prediction in anaerobic digestion systems. The framework integrates multivariate IoT sensing, physics-informed mathematical modeling of methane production dynamics grounded in thermodynamic gas relations, statistically validated machine learning regression, and lightweight Edge AI deployment using TinyML on ESP32 microcontrollers. It further incorporates on-device cumulative methane computation and cloud-synchronized visualization to enable autonomous, scalable, and cost-effective intelligent monitoring of biogas plants.
This repository accompanies the corresponding research work and is intended to support methodological transparency, reproducibility, baseline comparability, and future research extensions in AI-enabled renewable energy systems.

---

## 1. Background and Motivation

Anaerobic digestion is a bio-physico-chemical process governed by thermodynamic gas relations, microbial activity, and environmental interactions. Although methane production is the key performance indicator of a biodigester, it is rarely measured directly in conventional rural installations. Instead, operators rely on indirect observations such as flame quality, gas pressure build-up, or subjective experience. This lack of quantitative monitoring results in suboptimal operation, reduced methane yield, and potential safety risks.

From a process perspective, methane generation is influenced by tightly coupled variables including temperature, slurry moisture content, substrate composition, gas composition, and internal pressure. Variations in these parameters affect microbial stability, volumetric gas expansion, and overall digester efficiency. While advanced biochemical models such as mechanistic kinetic frameworks provide theoretical insight, they are computationally intensive and impractical for real-time embedded deployment in low-resource environments. On the other hand, purely data-driven machine learning approaches often treat methane production as a black-box problem, ignoring thermodynamic constraints and physical consistency.

This creates a methodological gap between physics-based process understanding and deployable artificial intelligence systems.

Recent advancements in the Internet of Things (IoT), edge computing, and TinyML enable machine learning inference directly on low-power microcontrollers. Such capability is particularly relevant for rural and decentralized renewable energy systems, where cloud connectivity may be unreliable and computational resources are limited. However, there remains limited research on integrating thermodynamics-based volumetric gas estimation with embedded machine learning for real-time methane rate prediction.

The motivation of this work is therefore twofold:

- To develop a physics-informed methane production estimation framework grounded in thermodynamic gas relations and real-time sensor measurements.
- To deploy a lightweight, statistically validated machine learning model on an ESP32 microcontroller for low-latency edge prediction of methane production rate (m<sup>3</sup>/h).

By combining physical modeling with data-driven intelligence at the edge, the proposed system aims to bridge the gap between renewable energy process modeling and practical, scalable deployment. This hybrid approach enhances interpretability, reduces computational overhead, and enables cost-effective intelligent monitoring suitable for the socio-economic context of Bangladesh and similar developing regions.

---

## 2. Research Contributions

This work makes the following key contributions:

- **Physics-Informed Methane Production Estimation Framework**  
   A thermodynamics-based volumetric methane estimation model is developed using the ideal gas law and pressure variation principles to compute real-time gas volume and methane fraction from multivariate sensor data.

- **Hybrid Physical–Data-Driven Modeling Approach**  
   The study integrates physics-based gas volume estimation with supervised machine learning to enhance predictive accuracy while maintaining physical interpretability, bridging the gap between mechanistic modeling and black-box regression.

- **Edge AI Deployment Using TinyML**  
   A Random Forest regression model is trained, statistically validated (RMSE, MAE, R$^2$), converted to a lightweight format, and deployed on an ESP32 microcontroller for real-time methane rate prediction (m<sup>3</sup>/h) without cloud dependency.

- **Real-Time Cumulative Methane Computation at the Edge**  
   A discrete-time integration method is implemented on-device to compute cumulative methane production, enabling continuous energy yield tracking.

- **Multivariate IoT-Based Monitoring Architecture**  
   A complete sensing framework is designed to measure internal and external temperature, humidity, gas concentrations (CH<sub>4</sub>, CO₂, H₂S), and pressure in real time, ensuring comprehensive environmental awareness of the digester.

- **Baseline Model Comparison and Statistical Validation**  
   The proposed model is rigorously evaluated against classical machine learning baselines using standard performance metrics (RMSE, MAE, R<sup>2</sup>) to ensure methodological transparency and reproducibility.

- **Low-Cost, Scalable Renewable Energy Monitoring Solution**  
   The proposed system demonstrates a cost-effective, deployable architecture suitable for rural and resource-constrained environments, contributing toward data-driven and sustainable biogas management.

Collectively, these contributions establish a hybrid physics-informed Edge AI framework that advances intelligent monitoring and methane rate prediction in decentralized anaerobic digestion systems.

---


## 3. Dataset Description

### 3.1. Dataset Title

**Physics-Informed Edge AI Biogas Monitoring Dataset: Environmental Parameters and Methane Production from Rural Digesters in Bangladesh**

---


### 3.2. Dataset Overview

This study utilizes a publicly available multivariate time-series dataset collected from rural anaerobic digester situated in Taljhari Agricultural Farm in Dhamoirhat Upazilla, Naogaon District in Rajshahi, Bangladesh. The dataset was developed to support physics-informed methane production estimation and embedded machine learning–based rate prediction.

The dataset captures real-time environmental, gas composition, and pressure parameters required for thermodynamic volumetric estimation and supervised regression modeling.

---

### 3.3. Data Acquisition

Sensor measurements were collected from operational biodigesters using IoT-based sensing modules. The monitoring system records both internal and external environmental variables to capture the dynamic behavior of methane production under real-world rural conditions.

Sampling was performed at regular time intervals to ensure temporal consistency for rate computation and cumulative integration.

---

### 3.4. Recorded Parameters

The dataset includes the following features:

- Internal temperature (°C)  
- Internal humidity (%)  
- External temperature (°C)  
- External humidity (%)  
- Gas pressure inside the digester (Pa or kPa)  
- Methane concentration (CH<sub>4</sub>, ppm)  
- Carbon dioxide concentration (CO₂, ppm)  
- Hydrogen sulfide concentration (H₂S, ppm)  
- Timestamp  

Using thermodynamic gas relations and pressure variation, the following derived variables are computed:

- Total gas volume change (ΔV)  
- Volumetric flow rate (m<sup>3</sup>/h)  
- Methane fraction  
- Methane production rate (m<sup>3</sup>/h)  

The methane production rate serves as the target variable for supervised learning.

---

### 3.5. Dataset Characteristics

- Type: Multivariate time-series dataset  
- Domain: Renewable energy / Anaerobic digestion  
- Region: Taljhari Agricultural Farm, Dhamoirhat, Naogaon, Rajshahi, Bangladesh
- Modeling Type: Physics-informed regression  
- Target Variable: Methane production rate, V_ch4 (m<sup>3</sup>/h)  

---
### 3.6. Data Availability

The dataset supporting this work is publicly available at:

GitHub Repository:
[Physics-Informed Edge AI Biogas Monitoring Dataset: Environmental Parameters and Methane Production from Rural Digesters in Bangladesh](https://github.com/luciferm0708/SmartBioGas-PIoT/tree/main/Dataset)

For long-term archival and citation purposes, the dataset will also be maintained with version control to ensure reproducibility of experimental results.

---

### 4. Preprocessing and Physics-Informed Derivation

The following preprocessing and physics-informed procedures were applied prior to model training:

#### 4.1 Sensor Calibration and Data Cleaning
1. Sensor calibration correction to compensate for systematic measurement bias  
2. Outlier detection and removal using threshold-based filtering  
3. Noise smoothing where necessary to reduce sensor fluctuations  


---


#### 4.2 Thermodynamics-Based Volumetric Derivation

To ensure physical consistency, methane production variables were derived using thermodynamic gas relations:

1. **Ideal Gas Law Application**

   The headspace gas volume relationship was expressed as:

   P V_hs = n R T

2. **Pressure-Based Volume Change**

   The change in gas volume due to pressure variation was computed as:

   ΔV = V_hs × (ΔP / P₀)

   where  
   ΔP = pressure difference between consecutive time steps  
   P₀ = reference pressure  
   V_hs = digester headspace volume  

3. **Volumetric Flow Rate Calculation**

   The gas flow rate was computed discretely:

   V̇ = ΔV / Δt

4. **Methane Fraction Estimation**

   Methane concentration (ppm) was converted to volumetric fraction:

   f = C_ppm / 10⁶

5. **Methane Production Rate (Target Variable)**

   The methane production rate was derived as:

   V_CH₄ = f × ΔV

   This derived methane volumetric flow (m<sup>3</sup>/h) was used as the supervised learning target.

---


#### 4.3 Feature Engineering and Alignment

6. Temporal alignment of multivariate time-series measurements  
7. Normalization of input features for regression stability  
8. Train–test split configuration for supervised learning evaluation  

All physics-informed derivation scripts, feature engineering modules, and preprocessing pipelines are provided in the `Code/` directory to ensure full experimental reproducibility and methodological transparency.
---
## 5. Model Architecture

### 5.1 Model Type

The methane production rate prediction model is based on a Random Forest regression framework.

---



### 5.2 Input Features

The model utilizes the following multivariate inputs:

- Internal temperature
- Internal humidity
- External temperature
- External humidity
- Gas pressure
  
---


### 5.3 Target Variable

The supervised learning target is the physics-informed methane production rate (m<sup>3</sup>/h), derived using thermodynamic volumetric equations.

---


### 5.4 Hyperparameters

The Random Forest model was trained using:

- `n_estimators = 10`  
- `max_depth = 5`  
- `min_samples_split = 2`  
- `min_samples_leaf = 1`  
- `n_jobs = -1`  
- `random_state = 42`  

---


### 5.5 Evaluation

The dataset was divided into training and testing subsets according to the predefined split configuration. Model performance was evaluated using Root Mean Squared Error (RMSE), Mean Absolute Error (MAE), Coefficient of Determination (R<sup>2</sup>).

- RMSE: 0.000340
- MSE: 0.000242
- R<sup>2</sup>: 0.9824

---


### 5.6 Edge Deployment

The trained model was optimized and deployed on an ESP32 microcontroller for real-time inference using [MicroMLGen](https://github.com/eloquentarduino/micromlgen) based on TinyML principles, ensuring low-latency prediction under embedded constraints.

---

### 6. License

This project is licensed under the Creative Commons Attribution–NonCommercial 4.0 International (CC BY-NC 4.0) License.

Commercial use is strictly prohibited without explicit permission from the authors.

---
### 7. Contact

For correspondence, data access requests, or academic collaboration:

Name: Faiyaz Khan Sami
B.Sc. in C.S.E.
Email: sami15-4910@diu.edu.bd || sami.khan.m.0107@gmail.com
