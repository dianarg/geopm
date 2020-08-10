# Acquire the source:
wget https://asc.llnl.gov/coral-2-benchmarks/downloads/AMG-master-5.zip

# Unpack the source:
unzip AMG-master-5.zip

# Change directories to the unpacked files:
cd AMG-master

# Patch MiniFE with the patch utility:
patch -p1 < ../0001-Adding-geopm-markup-to-CORAL-2-AMG.patch

# Build
make
