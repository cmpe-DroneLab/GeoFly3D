import subprocess
import os

def generate_orthophoto(project_name="project", resolution=5):
    cwd = os.getcwd()
    resolution = max(0.1, min(5, resolution))
    # options = " --auto-boundary --dem-resolution 1 --dsm --feature-quality high --force-gps --max-concurrency 2 --mesh-octree-depth 12 --min-num-features 12000 --pc-geometric --pc-quality high --ignore-gsd --feature-type dspsift"
    options = ""
    subprocess.run("docker run -ti --rm -v " + cwd + ":/projects opendronemap/odm --project-path /projects " + str(project_name) +" --orthophoto-resolution " + str(resolution) + options, shell=True)

