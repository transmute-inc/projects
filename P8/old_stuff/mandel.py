import numpy as np

def mandelbrot_set(width, height, zoom=1, x_off=0, y_off=0, niter=256):
    w,h = width, height
    pixels = np.arange(w*h, dtype=np.uint16).reshape(h, w)
    for x in range(w): 
        for y in range(h):
            zx = 1.5*(x + x_off - 3*w/4)/(0.5*zoom*w)
            zy = 1.0*(y + y_off - h/2)/(0.5*zoom*h)
            z = complex(zx, zy)
            c = complex(0, 0)
            for i in range(niter):
                if abs(c) > 4: break
                c = c**2 + z
            color = (i << 21) + (i << 10)  + i * 8
            pixels[y,x] = color
