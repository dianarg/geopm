# Acquire the source:
wget https://asc.llnl.gov/CORAL-benchmarks/Throughput/MiniFE_ref_2.0-rc3.tar.gz

# Unpack the source:
tar zxvf MiniFE_ref_2.0-rc3.tar.gz

# Change directories to the unpacked files.
cd miniFE_openmp-2.0-rc3

# Patch MiniFE with the patch utility:
patch -p1 < ../0001-Adding-geopm-markup-to-CORAL-version-of-miniFE.patch

# Build
cd src
make
