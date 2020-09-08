# 2021NutLib

2021学ロボ用 STM32ライブラリ

# 概要

2021学ロボ用に開発しているSTM32マイコン対象のヘッダオンリーライブラリです。
gcc version 7.2.1(GNU Tools for Arm Embedded Processors 7-2017-q4-major)を使用して開発しています。
c++17ビルド環境であれば使えます(多分)。
その他[esa参照](https://nagaokaroboconproject.esa.io/posts/61)

# 依存関係

- 2020/9現在最新のHALライブラリ
- 2020/9現在最新のEigenライブラリ

# 使用方法

まずcubeMXなりcubeIDEなりでプロジェクトを生成してください。
この時ピンアサインやAFの設定は各ユーザーが行ってください。

生成したらプロジェクトをc++に設定変更してください([esa参照](https://nagaokaroboconproject.esa.io/posts/62))。
その後このリポジトリのNutLibをプロジェクトのインクルードファイル下にコピーするか,
ビルド設定でNutLibまでインクルードパスを通してください。
Eigenについても同様です。

## コーディング

各インスタンスの生成はユーザーが行ってください。

使用にあたって`nut::TimeSchedulerBase::TimeCheck()`は必ず呼び出してください。
また各通信による受信処理を行うもの(コントローラやセンサ等)は各受信メンバ関数をユーザーが割り込み関数内に記述してください。

コードの詳しいことは[2021NutLib_UseSample](https://gitlab.com/robopro_nut/2021nhkrobocon/2021nutlib_usesample)を参照するか、[リファレンス](https://robopro_nut.gitlab.io/2021nhkrobocon/2021nutlib/index.html)を確認してください。


### 注意

gcc version 7以前のコンパイラを使用する場合,c++17ビルドオプション(-std=c++17 または -std=c++1z)を追加してください。コンパイルが通りません。

NutLibをプロジェクトのインクルードファイル下にコピーしてプロジェクトフォルダに含めると、エディタがエラーメッセージを吐くことがありますが特に問題ありません。コンパイルは通ります。
