# grasp_data_generator
此仓库用来根据多视角点云生成抓取数据集，主要包括两个功能：
1. 多视角点云生成grasp predict数据集
2. 根据生成grasp predict数据集进一步生成grasp classify数据集

## 环境
- vtk
- pcl
- Eigen3 3.3.4

## 编译
```bash
cd grasp_data_generator
cmake .
make -j4
```

## 运行
```bash
./generate_predict_dataset --data_dir=/home/waha/workspace/data/ --cfg_dir=./config/generate_parameter.cfg
```

## docker
```bash
docker image build -t grasp_generator .
```