# Usage:
1. Make your own quick file with a main that calls one of the visualization functions:
```
# myfile.py

if __name__ == '__main__':
    animate_robot('package://my_package/ur5.yaml', 'package://my_package/ur5_path.yaml')
```
2. Open blender from the current terminal (`blender`).
3. Open the [python console](https://docs.blender.org/manual/en/dev/editors/python_console.html) from the blender window.
4. In the python console, type the following:
 ```
 filename = 'myfile.py'
 exec(compile(open(filename).read(), filename, 'exec'))
 ```
 
5. Repeat while you develop the script outside of blender
    (Blender's python editing isn't really worth it.)
