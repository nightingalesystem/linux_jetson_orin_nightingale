# Kernel_sources_XavierNX

Contains Airvolute customized kernel sources for Xavier NX 


# Use kernel sources 

  - clone repo to .../Linux_for_tegra directory 
  - switch branch to corresponding JetPack version 



# Create support for new JP version
  - clone repo to .../Linux_for_tegra/  
  - create new git branch 
  - sync sources with desired tag
  - copy files from .../Linux_for_tegra/source -> 
  - deinit all git sub-directories  find . -type d -name ".git" -exec rm -rf {} +


