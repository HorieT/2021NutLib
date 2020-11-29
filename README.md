# 2021NutLib

2021学ロボ用 STM32ライブラリ
(執筆：堀江智陽 and more...の予定？？？)

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
~~ビルド設定でNutLibまでインクルードパスを通してください。~~
__仕様変更(20/11/29)__
必ずコピー配置してください。リンカオブジェクトエラーします。

Eigenについても同様です。

## コーディング

各インスタンスの生成はユーザーが行ってください。

~~使用にあたって`nut::TimeSchedulerBase::TimeCheck()`は必ず呼び出してください。~~
~~また各通信による受信処理を行うもの(コントローラやセンサ等)は各受信メンバ関数をユーザーが割り込み関数内に記述してください。~~

__仕様変更(20/11/29)__
使用頻度の高いorライブラリ標準で使用するコールバックハンドラはNutLib側で吸収しました。
`HALCallback`フォルダ下に実体が定義されています。リファレンスでは`nut::callback`名前空間を参照してください。
ユーザがコールバック関数内に関数記述したい場合はインライン定義された`nut::HALCallback`オブジェクトに`nut::HALCallback::AddCallback()`または`nut::HALCallback::AddExclusiveCallback()`してください。また戻り値イテレータを使用して関数の削除が可能です。

使用例

``` c++
/* HAL_UART_RxHalfCpltCallback()が呼ばれるたびに最優先でPA0ピンの出力をトグルする */
nut::callback::UART_RxHalfComplete.AddCallback(0, [](UART_HandleTypeDef *huart){HAL_GPIO_TogglePin(GPIOA, GPIO_Pin_0);});
```

またどうしてもNutLib側で吸収されたコールバック関数を自己記述したい場合は`#define UNUSE_NUTLIB_CALLBACKS`してください.
この時、`HALCallback`フォルダ下の実装のように`nut::HALCallback::ReadCallbacks()`を記述しないとライブラリは完全に動作しません。


コードの詳しいことは ~~[2021NutLib_UseSample](https://gitlab.com/robopro_nut/2021nhkrobocon/2021nutlib_usesample)を参照するか、~~ [リファレンス](https://robopro_nut.gitlab.io/2021nhkrobocon/2021nutlib/index.html)を確認してください。(ローカルではpublic/index.html)

新しく使用方法サンプルを書きます(そのうち)
それかその他最新ファームウェアを参考にしてください。

また要望があれば堀江がサンプルコードを書きます

### 注意

gcc version 7以前のコンパイラを使用する場合,c++17ビルドオプション(-std=c++17 または -std=c++1z)を追加してください。コンパイルが通りません。

NutLibをプロジェクトのインクルードファイル下にコピーしてプロジェクトフォルダに含めると、エディタがエラーメッセージを吐くことがありますが特に問題ありません。コンパイルは通ります。そういうときはIndex->Rebuiltをしてみましょう。
