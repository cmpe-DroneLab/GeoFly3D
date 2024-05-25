import os 
from pyodm import Node

n = Node('localhost', 3000)
print('creating task')
task = n.create_task(['/home/netlab/Desktop/Workspace/output/P2190236.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2190237.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2190238.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2190239.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2190240.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2190241.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2200242.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2210243.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2210244.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2210245.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2210246.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2210247.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2210248.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2220249.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2230250.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2230251.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2230252.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2230253.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2230254.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2240255.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2250256.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2250257.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2250258.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2250259.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2250260.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2250261.JPG',
                      '/home/netlab/Desktop/Workspace/output/P2260262.JPG',

                    
                      ],
                      {'auto-boundary': True, 'dem-resolution': 1.0, 'dsm': True, 'orthophoto-resolution': 1.0, 'pc-quality': 'high'},
                      progress_callback=print)
print('waiting for task')
task.wait_for_completion(print)
print('comlpeted task')
print(os.listdir(task.download_assets("results"))[0:2])

