# AR Model

Bases on ORB feature to build AR Model，this project first try to find the platform then project 3D model on it. The final result is showing below：

[The link](https://github.com/zlf1993/ar_model/blob/master/res.gif)

## Reference

[](https://www.jianshu.com/p/ed57ee1056ab)
[](https://www.jianshu.com/p/f7f3e7b6ebf5)
[](https://blog.csdn.net/yc461515457/article/details/48625621)
[](http://manaai.cn/index3.html?page=3)

## Prepare for background picture

In ar_main.py there is bachgroundcard.jpg, please replay it with your own background picture.

## Main methods

Extract Fast(Features from Accelerated Segment Test) and rotation brief feature.
1. Select a pixel P from the picture. Then we will determine whether it is a feature point. We first set its brightness value to Ip.
2. Set an appropriate threshold t.
3. Consider a discrete Bresenham circle with a radius equal to 3 pixels centered on the pixel point. There are 16 pixels on the boundary of the circle (as shown in Figure below).
[](https://github.com/zlf1993/ar_model/blob/master/)
![avatar](/pictures/FAST.png1.png)
Figure 1. Schematic diagram of FAST feature points
4. Now, if there are n consecutive pixels on this 16-pixel circle, their pixel values ​​are either larger than Ip + t or smaller than Ip−t, then it is a corner point.



## 将目标标识出来

做到这里还远远不够。我们最好将目标用方框标识出来，这样方便我们后续进行平面映射。简单的来说，这个步骤是最难得部分，我们需要根据模板图片，找到相对于目标图片的映射关系，进而确定一个转换的矩阵，这个操作叫做Homography, 单应性矩阵。

```
src_pts = np.float32([kp_model[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
dst_pts = np.float32([kp_frame[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)
# compute Homography
M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
h, w = model.shape[0], model.shape[1]
pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)
# project corners into frame
dst = cv2.perspectiveTransform(pts, M)  
# connect them with lines
img2 = cv2.polylines(frame, [np.int32(dst)], True, 255, 3, cv2.LINE_AA)
```

上面步骤是在检测到 模板图片的关键点、每一帧的关键点的基础上做的。接下来要实现的是如何把我们的3D模型放到平面上去了。

## 最后一步

这一步要解决两个问题：

1. 根据单应性矩阵和相机外参，得到矫正的坐标系，并把OBJ立体模型放置上面；
2. 如何解析OBJ？

![](https://bitesofcode.files.wordpress.com/2017/09/selection_003.png)

如果上图所示，通常情况下，相机的外参都会有四个参数。三维坐标系的三个坐标，以及一个旋转角度。我们可以把z去掉，也就是把R3拿掉，因为我们是要把OBJ投影到一个平面内，那么z就自然都是0了。

![](https://bitesofcode.files.wordpress.com/2018/07/selection_017.png)

接着是十分复杂的一顿运算，有兴趣的朋友可以推导一下，如果将3D的模型投影到平面上。总之我们直接用代码来展示一下最终效果：

```
def projection_matrix(camera_parameters, homography):
    """
    From the camera calibration matrix and the estimated homography
    compute the 3D projection matrix
    """
    # Compute rotation along the x and y axis as well as the translation
    homography *= -1
    rot_and_transl = np.dot(np.linalg.inv(camera_parameters), homography)
    col_1 = rot_and_transl[:, 0]
    col_2 = rot_and_transl[:, 1]
    col_3 = rot_and_transl[:, 2]
    # normalise vectors
    l = math.sqrt(np.linalg.norm(col_1, 2) * np.linalg.norm(col_2, 2))
    rot_1 = col_1 / l
    rot_2 = col_2 / l
    translation = col_3 / l
    # compute the orthonormal basis
    c = rot_1 + rot_2
    p = np.cross(rot_1, rot_2)
    d = np.cross(c, p)
    rot_1 = np.dot(c / np.linalg.norm(c, 2) + d / np.linalg.norm(d, 2), 1 / math.sqrt(2))
    rot_2 = np.dot(c / np.linalg.norm(c, 2) - d / np.linalg.norm(d, 2), 1 / math.sqrt(2))
    rot_3 = np.cross(rot_1, rot_2)
    # finally, compute the 3D projection matrix from the model to the current frame
    projection = np.stack((rot_1, rot_2, rot_3, translation)).T
    return np.dot(camera_parameters, projection)
```

最后你就可以从视频流中，给你的目标物体拼接上一个可爱的小动物。
![](https://i.loli.net/2018/09/18/5ba0f548942da.gif)

## 代码使用

将自己的模板图片替换掉reference下面的图片，运行：

```
python3 ar_main.py
```

```
本教程由奇异AI提供，一个专注黑科技布道以及算法交易的平台
微信搜索【奇异人工智能】获取第一时间资讯
官网：strangeai.pro
```
