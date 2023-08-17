import pandas as pd

#任意的多组列表
a = [1,2,3]
b = [4,5,6]    





heightPerturbationRange = 0.025
heightfieldSource=1

if heightfieldSource==1:
  numHeightfieldRows = 256
  numHeightfieldColumns = 256
  ones_c=[1.0]*numHeightfieldColumns
  heightfieldData = [0]*numHeightfieldRows*numHeightfieldColumns 
  for j in range (int(numHeightfieldColumns)):
      
      heightfieldData[0+j*numHeightfieldRows:numHeightfieldColumns+j*numHeightfieldRows]=[j*0.01]*numHeightfieldColumns

#字典中的key值即为csv中列名
dataframe = pd.DataFrame({'a_name':heightfieldData})

#将DataFrame存储为csv,index表示是否显示行名，default=True
dataframe.to_csv("test.csv",index=False,sep=',')   


with open("test.txt","w") as f:
    f.write(str(heightfieldData))  # 自带文件关闭功能，不需要再写f.close()