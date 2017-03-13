# squirrel_prediction repository


Repository for prediction and learning related SQUIRREL packages.

Installation requiremets
```
rosdep install --from-path squirrel_prediction -i -y
```
Running  the service:
===============
```
rosrun squirrel_relations_prediction predict_relations_server.py 
```
Service call for relations prediction:
===============

```
# input parameters
string data_path
string input_file
string output_file
int32 number_of_columns
---
#output
bool finished
```


