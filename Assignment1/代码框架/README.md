已完成提高题的get_rotation函数，并在main函数里执行了调用。

有一个小问题：绕任意轴旋转时，画出点的像素下标会超出framebuffer的范围。
            所以我在rasterizer.cpp中的setpixel函数中加了越界的判断