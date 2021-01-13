# Extended Kalman Filter
---
## Rubric Points
---
Here I will consider [the rubic points](https://review.udacity.com/#!/rubrics/748/view) individually and describe how I addressed each point in my implementation.

1. RMSE of output coordinates is below threshold  

    Your algorithm will be run against Dataset 1 in the simulator which is the same as "data/obj_pose-laser-radar-synthetic-input.txt" in the repository. We'll collect the positions that your algorithm outputs and compare them to ground truth data. Your px, py, vx, and vy RMSE should be less than or equal to the values [.11, .11, 0.52, 0.52].

1. Sensor Fusion algorithm follows the general processing flow.  
    1. Represented by two-dimensional position and two-dimensional speed  
    

    ```cpp
        ekf_.x_ = VectorXd(4);
    ```

    1. 特定のセンサから新しい測定値を取得  
    1. 初回の場合は単に状態と共分散行列を初期化する
    1. 2回目以降では「予測」と「測定の更新」を行う
    1. 予測フェーズ
        1. 予測の前に今回の観測までの経過時間を計算するΔt
        1. 経過時間から新しい状態遷移を計算 EKF, F, Q
        1. 誤差の共分散行列Pと予測推定値xを計算する
    1. 更新フェーズ
        1. センサーのタイプに合わせて測定更新を分ける
        1. レーダーの観測値では新しいヤコビアンHjを計算して非線形測定関数を使用して予測状態を投影し観測値の更新する
        1. レーザーの観測値ではレーザーの観測モデルH行列と共分散行列R行列を使用してEKFを設定した後で観測値の更新を行う


1. Kalman Filter algorithm handles the first measurements appropriately.  

    Your algorithm should use the first measurements to initialize the state vectors and covariance matrices.

1. Kalman Filter algorithm first predicts then updates.  

    Upon receiving a measurement after the first, the algorithm should predict object position to the current timestep and then update the prediction using the new measurement.

1. Kalman Filter can handle radar and lidar measurements.  

    Your algorithm sets up the appropriate matrices given the type of measurement and calls the correct measurement function for a given sensor type.
