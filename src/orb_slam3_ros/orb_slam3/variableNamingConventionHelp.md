## Variable Naming Convention in ORB SLAM3

At a glance, variables names are completely uninteligable so this is a guide to help decode what they mean.

Generally, the structure is `<is_member_variable> <type> <is_pointer> <description>`
Where:
```
is_member_variable := 
  | m - is member variable
  | ε - not member variable

type :=
  | n - number
  | f - float
  | v - std vector
  | s - std set
  | t - thread
  | T - eigen transformation matrix
  | V - eigen vector
  | ...

is_pointer :=
  | p - is a pointer
  | ε - is not a pointer
```

Obviously this is not always followed, but hopefully this gives you the right idea when looking at a stupid variable name like `mVwbBefGBA`!

#### Example 1.

`Sophus::SE3f mTcwBefGBA;`
```
m:    member variable 
T:    transformation 
cw:   camera to world 
bef:  before 
GBA:  global bundle adjustment 
```


#### Example 2.

`const int mnGridCols;`
```
m:        member variable
n:        number
GridCols: description
```

#### Example 3.

`std::vector<KeyFrame *> mvpKeyFrameOrigins;`
```
m:               member variable
v:               std vector
p:               pointer
KeyFrameOrigins: description
```