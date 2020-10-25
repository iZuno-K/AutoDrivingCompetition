# AutoDrivingCompetition

## Original scenario
```bash
$ roslaunch aichallenge_bringup aichallenge_bringup.launch
```
### Scenario
```bash
$ roscd aichallenge_bringup/scenario/
$ python3 scenario.train.py 

```


## 交差点専用シナリオの追加
### 静止車両通過後のroslaunch
```bash
$ roslaunch aichallenge_bringup intersection1.launch
```
### Scenario
```bash
$ roscd aichallenge_bringup/scenario/
$ python3 intersection1.py 
```
