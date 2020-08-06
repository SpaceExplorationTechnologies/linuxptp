#!/bin/bash
#
# Creates a debian package of the linuxptp server.
#
# Call this using:
# ./create_package.sh build_number
#
# If build_number is omitted, 'dev' will be used.
#
set -e

# Set version and architecture
VERSION="$(./version.sh)-${1:-dev}"
ARCH="$(dpkg --print-architecture)"
echo "Building package for $ARCH $VERSION"

# Generate control file
sed -e "s/^Version:.*/Version: ${VERSION}/"        \
    -e "s/^Architecture:.*/Architecture: ${ARCH}/" \
    control.in > control

# Build and install
make install DESTDIR=debian

# Add debian scripts
install -D -m 644 control        debian/DEBIAN/control
install -D -m 644 init/ptp4l.cfg debian/etc/ptp4l.cfg
install -D -m 755 init/ptp4l     debian/etc/init.d/ptp4l
install -D -m 644 init/defaults  debian/etc/default/ptp4l
install -D -m 755 init/if-updown debian/etc/network/if-up.d/ptp4l
install -D -m 755 init/if-updown debian/etc/network/if-post-down.d/ptp4l

echo /etc/ptp4l.cfg                    >  debian/DEBIAN/conffiles
echo /etc/default/ptp4l                >> debian/DEBIAN/conffiles
echo /etc/network/if-up.d/ptp4l        >> debian/DEBIAN/conffiles
echo /etc/network/if-post-down.d/ptp4l >> debian/DEBIAN/conffiles

# Create debian package
fakeroot dpkg-deb --build debian
mv debian.deb "linuxptp-sx_${VERSION}.deb"
