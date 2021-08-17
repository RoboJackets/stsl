#! /usr/bin/env bash

set -e 

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

case "$1" in
  --patch)
    part="PATCH"
    echo "Bumping patch version..."
    ;;

  --minor)
    part="MINOR"
    echo "Bumping minor version..."
    ;;

  --major)
    part="MAJOR"
    echo "Bumping major version..."
    ;;

  *)
    echo "Unsupported request ($1). Please use --major, --minor, or --patch."
    exit 1
    ;;
esac

package_manifests=($(git ls-files $SCRIPT_DIR/.. | grep package.xml))

version_tag=$(grep "<version>.*</version>" ${package_manifests[0]})
version=$(echo "$version_tag" | sed -e 's/<[^>]*>\|  //g' )

echo "Old version: $version"

semver=(${version//./ })
major="${semver[0]}"
minor="${semver[1]}"
patch="${semver[2]}"

case "$part" in
  MAJOR)
    ((major=major + 1))
    minor=0
    patch=0
    ;;

  MINOR)
    ((minor=minor + 1))
    patch=0
    ;;

  PATCH)
    ((patch=patch + 1))
    ;;
esac

new_version=$major.$minor.$patch

echo "New version: $new_version"

for manifest in ${package_manifests[@]}
do
  sed -i "s/<version>.*<\/version>/<version>$new_version<\/version>/g" $manifest
done

read -p "Do you want to commit and tag this version bump? (y/N)" should_commit
if [[ $should_commit =~ ^[Yy]$ ]]
then
  git commit -a -m "Bumping version to $new_version"
  git tag $new_version
else
  echo "Not commit made."
fi
