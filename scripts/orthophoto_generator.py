import subprocess
import os

def generate_orthophoto(project_name="project", resolution=5):
    cwd = os.getcwd()
    resolution = max(0.1, min(5, resolution))
    subprocess.run("docker run -ti --rm -v " + cwd + ":/projects opendronemap/odm --project-path /projects " + str(project_name) +" --skip-3dmodel --orthophoto-resolution " + str(resolution), shell=True)

