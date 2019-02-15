# PID
## Description
注1: STM32的DSP库中也有PID函数. 

### 模糊PID
模糊表:

行:EC, 列:E

|Kp | NL | NM | NS | ZE | PS | PM | PL|
|---|:---|:---|:---|:---|:---|:---|:--|
|NL | PL | PL | PM | PM | PS | PS | ZE|
|NM | PL | PL | PM | PM | PS | ZE | ZE|
|NS | PM | PM | PM | PS | ZE | NS | NM|
|ZE | PM | PS | PS | ZE | NS | NM | NM|
|PS | PS | PS | ZE | NS | NS | NM | NM|
|PM | ZE | ZE | NS | NM | NM | NM | NL|
|PL | ZE | NS | NS | NM | NM | NL | NL|

|Ki | NL | NM | NS | ZE | PS | PM | PL|
|---|:---|:---|:---|:---|:---|:---|:--|
|NL | NL | NL | NL | NM | NM | ZE | ZE|
|NM | NL | NL | NM | NM | NS | ZE | ZE|
|NS | NM | NM | NS | NS | ZE | PS | PS|
|ZE | NM | NS | NS | ZE | PS | PS | PM|
|PS | NS | NS | ZE | PS | PS | PM | PM|
|PM | ZE | ZE | PS | PM | PM | PL | PL|
|PL | ZE | ZE | PS | PM | PL | PL | PL|

|Kd | NL | NM | NS | ZE | PS | PM | PL|
|---|:---|:---|:---|:---|:---|:---|:--|
|NL | PS | PS | ZE | ZE | ZE | PL | PL|
|NM | NS | NS | NS | NS | ZE | NS | PM|
|NS | NL | NL | NM | NS | ZE | PS | PM|
|ZE | NL | NM | NM | NS | ZE | PS | PM|
|PS | NL | NM | NS | NS | ZE | PS | PS|
|PM | NM | NS | NS | NS | ZE | PS | PS|
|PL | PS | ZE | ZE | ZE | ZE | PL | PL|

... 
## Usage
初始化PID结构体: `void pid_setup_f32(PID_f32 *pid);`

复位PID状态: `void pid_reset_f32(PID_f32 *pid);`

### 模糊PID算法
初始化PID结构体: `FuzzyPIDInit(FUZZYPID *pid, float32_t tar, float32_t maxdKp, float32_t mindKp, float32_t qKp, float32_t maxdKi, float32_t mindKi, float32_t qKi, float32_t maxdKd, float32_t mindKd, float32_t qKd);`

运行PID算法: `FuzzyPID(FUZZYPID *pid, float32_t cur)`

## Reference
...

