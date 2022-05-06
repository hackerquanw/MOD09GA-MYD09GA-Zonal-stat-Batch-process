# MOD09GA-MYD09GA-Zonal-stat-Batch-process
# Programming language: Python
# 编写背景
- MODIS反射率产品是计算地球地表反照率过程中最常用的数据，分为MOD09GA与MYD09GA两种，分别对应terra与aqua卫星。MOD09GA/MYD09GA产品的时间分辨率为天，空间分辨率为1000m，波段范围是1-7波段，产品的数据格式为hdf4格式。重投影为UTM后，每个波段为1个tif文件，由于MODIS数据时间分辨率很高，长时间环境变化分析需要处理非常多的文件，例如每天1景数据，20年就需要超过7000景，全波段提取将超过50000个文件。使用者的研究区域各不相同，需要按照指定区域来提取反射率值，因此，批量提取是必须采取的措施。本软件将实现MOD09GA数据集预处理后的各波段TIF文件地表反射率值的批量区域统计，本文档以MOD09GA数据预处理为例，同样适用于MYD09GA数据。
# 设计原理
下图是本软件的运行流程示意图
![image](https://user-images.githubusercontent.com/44941550/167151340-7bf83b79-4e6c-45b5-93e1-020d07315e56.png)

# 操作步骤

（1）运行Anaconda中的Jupyter notebook 

（2）创建Python3代码

（3）准备好输入文件
将需要预处理的MOD09GA数据文件放到一个文件夹中，路径仅有英文、数字和下划线这三种类型的字符。

（4）将代码输入框中并修改代码

①在第124行源代码在这里设置统计量指标，有如下几种选择average, mean, max, min, median, sum, std, unique。

②在第165-167行源代码输入区域shp文件路径、输入待提取tif文件夹路径、输出文件夹路径。
