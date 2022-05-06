import numpy
from osgeo import gdal,osr,ogr
import shapefile
import sys
from pandas.core.frame import DataFrame
import os
import time
  
def search_f(inpath):
    # 检索待处理影像
    F_list = []
    for root,dir,fields in os.walk(inpath):
        for field in fields:
            if field.endswith("tif") is True:
                f_fn = os.path.join(inpath, field)
                F_list.append(f_fn)

    return F_list

def Getshp_FID(shp_fn):
    """获取要素FID"""

    # 用pyshp库打开矢量
    file = shapefile.Reader(shp_fn)

    # 获取要素属性、要素数量
    features = file.records()
    features_number = len(file.records())

    # 获取要素FID
    FID_list = []
    Feature_name_list = []
    for i in range(features_number):

        feature = str(features[i])

        # 从Feature中获取FID
        feature_id = int(feature.split("#")[1].split(":")[0])
        FID_list.append(feature_id)

        # 从Feature中获取FID对应的数字
        feature_name = eval(str(features[i]).split(":")[1].split("[")[1].split("]")[0])
        Feature_name_list.append(feature_name)

    return FID_list, Feature_name_list, features_number


def zonal_stats(feat, input_zone_polygon, raster_paths):
    """统计shp图中要素反射率"""

    # 创建空列表，用于存储计算后的反射率值
    zonal_average = []

    # 打开影像
    raster = gdal.Open(raster_paths)

    # 打开图层
    shp = ogr.Open(input_zone_polygon)
    lyr = shp.GetLayer()

    # Get raster georeference info
    transform = raster.GetGeoTransform()
    band = raster.RasterCount

    # 左上角坐标；影像分辨率
    xOrigin = transform[0] #The upper left corner of the upper left pixel is at position (padfTransform[0],padfTransform[3])
    yOrigin = transform[3]
    pixelWidth = transform[1] #[1] is the pixel width
    pixelHeight = transform[5] #[5] is the pixel height

    """
    矢量重投影为与栅格相同的投影
    CoordinateTransformation 坐标转换
    """
    sourceSR = lyr.GetSpatialRef()
    targetSR = osr.SpatialReference()
    targetSR.ImportFromWkt(raster.GetProjectionRef())
    coordTrans = osr.CoordinateTransformation(sourceSR, targetSR)
    # feat = lyr.GetNextFeature() #这一句如果不注释掉，就会只提取第一个图斑
    geom = feat.GetGeometryRef()
    geom.Transform(coordTrans)

    # Get extent of feat
    geom = feat.GetGeometryRef()
    if (geom.GetGeometryName() == "MULTIPOLYGON"):
        count = 0
        pointsX = []
        pointsY = []
        for polygon in geom:
            geomInner = geom.GetGeometryRef(count)
            ring = geomInner.GetGeometryRef(0)
            numpoints = ring.GetPointCount()
            for p in range(numpoints):
                lon, lat, z = ring.GetPoint(p)
                pointsX.append(lon)
                pointsY.append(lat)
            count += 1
    elif (geom.GetGeometryName() == 'POLYGON'):
        ring = geom.GetGeometryRef(0)
        numpoints = ring.GetPointCount()
        pointsX = [] #矢量图斑每一个点的坐标x
        pointsY = [] #矢量图斑每一个点的坐标y
        for p in range(numpoints):
            lon, lat, z = ring.GetPoint(p)
            pointsX.append(lon)
            pointsY.append(lat)

    else:
        sys.exit("ERROR: Geometry needs to be either Polygon or Multipolygon")

    xmin = min(pointsX) #每个图斑x坐标最小值
    xmax = max(pointsX)
    ymin = min(pointsY)
    ymax = max(pointsY)

    print('xmin,xmax,ymin,ymax',xmin,xmax,ymin,ymax)

    # Specify offset and rows and columns to read
    xoff = int((xmin - xOrigin) / pixelWidth) #以栅格左上角为原点，每个图斑左上角位置跟栅格原点的距离（分子是坐标距离m，除以分母后得到栅格行列距离
    yoff = int((yOrigin - ymax) / pixelWidth)

    xcount = int((xmax - xmin) / pixelWidth) + 1 #一个shp所有图斑的区域范围有多少个栅格行列
    ycount = int((ymax - ymin) / pixelWidth) + 1
    
    # print('xoff,yoff',xoff,yoff)
    # print('xcount,ycount',xcount,ycount)
    
    # Create memory target raster 其中gdal.GDT_Float32 或者gdal.GDT_Byte 取决于元数据里面的像元值格式band的type，但是不一致也能提取
    target_ds = gdal.GetDriverByName("MEM").Create('', xcount, ycount, 5, gdal.GDT_Float32)
    target_ds.SetGeoTransform((xmin, pixelWidth, 0, ymax, 0, pixelHeight,))

    # Create for target raster the same projection as for the value raster
    raster_srs = osr.SpatialReference()
    raster_srs.ImportFromWkt(raster.GetProjectionRef())
    target_ds.SetProjection(raster_srs.ExportToWkt())

    # Rasterize zone polygon to raster
    raster_bandlist = list(range(band, 0, -1))
    b_v = [1]*band
    gdal.RasterizeLayer(target_ds, raster_bandlist, lyr, burn_values=b_v)

    for i in range(band):
        banddataraster = raster.GetRasterBand(i + 1)
        dataraster = banddataraster.ReadAsArray(xoff, yoff, xcount, ycount).astype(numpy.float)
        bandmask = target_ds.GetRasterBand(i + 1)
        datamask = bandmask.ReadAsArray(0, 0, xcount, ycount).astype(numpy.float)

        # Mask zone of raster
        zoneraster = numpy.ma.masked_array(dataraster, numpy.logical_not(datamask))

        # Calculate statistics of zonal raster
        zonal_average.append(numpy.average(zoneraster)) #在这里设置统计量指标average,mean,max,min,median,sum,std,unique
        print('zoneraster',zoneraster)
    return zonal_average


def get_vi_ref(input_zone_polygon, raster_paths, Ref_out):

    """统计矢量范围均值"""

    # 第一步：获取要素FID和对应图斑的列表
    FID_list, Feature_name_list, features_number = Getshp_FID(input_zone_polygon)
    shp = ogr.Open(input_zone_polygon)
    lyr = shp.GetLayer()

    # 第二步：获取反射率Ref
    Ref = []

    for FID in FID_list:
        feat = lyr.GetFeature(FID)
        Ref_meanValue = zonal_stats(feat, input_zone_polygon, raster_paths)

        # 获取各波段反射率值，如果有多个波段就在这里加
        b1 = Ref_meanValue[0]  # ref
        print('raster_paths=',raster_paths)
        # 追加至列表
        Ref.append(Ref_meanValue)


    # 第三步：Ref VI 输出dataframe 用法：https://blog.csdn.net/cymy001/article/details/78275886
    ref1=raster_paths.split('/')[1].split('-')[0]+'-'+raster_paths.split('-')[1]+'-'+raster_paths.split('-')[2]+'-'+raster_paths.split('-')[-2]+'-'+raster_paths.split('-')[-1] #按日期和数据子集编号命名字段名称，以便组合字段后能区分不同日期和子集的数据
    Ref = DataFrame(Ref, columns=[ref1], index=FID_list) #如果有多个波段就在columns添加

    # Ref VI 输出为CSV
    Ref.to_csv(Ref_out)
    print("提取完成，图斑数量为：", features_number)

    return


def main(input_zone_polygon,Ref_vi_out,raster_path):

    fn_list =  search_f(raster_path)

    for f in fn_list:
        print("提取", f)
        # 以影像日期为csv名
        Ref = f + "_ref.csv" #这里设置输出的文件名

        Ref_path = os.path.join(Ref_vi_out, Ref) # path的用法参考 https://www.cnblogs.com/an-ning0920/p/10037790.html

        get_vi_ref(input_zone_polygon, f, Ref_path)
########################
"""
split函数 f.split("\\")[-1].split(".")[0].split("_")[2] + "_ref.csv"
("\\")[-1] 将\替换成\\ 并取最后一段字符
.split(".")[0]按照.打断，并取第一段字符
.split("_")[2]按照_打断，并取第三段字符
提取两段数组用加号将两行.split代码连起来
高分一号示例：
GF1_WFV1_E101.8_N24.7_20180521_L1A0003208108_rpcortho_rad_subset_flaash.tif
下面这句就可以提取高分一号日期和日期后那一段
Ref = f.split("\\")[-1].split(".")[2].split("_")[1] + f.split("\\")[-1].split(".")[2].split("_")[2] + "_ref.csv"
f是每一个文件的路径和文件名：D:\3Sproject\3Sdata\GF1\RSDgeotrue\GF1_WFV1_E101.8_N24.7_20180521_L1A0003208108_rpcortho_rad_subset_flaash.tif
之前的代码：Ref = f.split("\\")[-1].split(".")[0].split("_")[3:5] + "_ref.csv"
上面这行代码存在问题，不适合GF1的提取
"""
#########################
 
if __name__ == '__main__':
    start = time.time()
    input_zone_polygon = r"D:\3Sproject\taskdata\2021\3lakeChl\allsamplepolygenrepBuff100.shp"#研究区shp图
    Ref_vi_out = r"D:\3Sproject\3Sdata\MODIS\MOD09GA\UTMtif/"#ref输出路径
    raster_path = r"D:\3Sproject\3Sdata\MODIS\MOD09GA\UTMtif/"#待提取影像路径
    main(input_zone_polygon, Ref_vi_out, raster_path)
    end = time.time()
    print('deal spend: {s} s'.format(s = end-start))
