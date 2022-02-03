/**
 * Updater used to update a python version file (i.e. `version.py`).
 */

/**
 * Reads the current version from the Python configuration file.
 */
module.exports.readVersion = function (contents) {
  // console.debug(`file contents:\n\n${contents}`);
  const version = contents.match(/(?<=version = ")\d+\.\d+\.\d+(?=")/g)[0];
  // console.debug("found version:", version);
  return version;
};

/**
 * Writes the new version to the Python configuration file.
 */
module.exports.writeVersion = function (contents, version) {
  // console.debug(`file contents:\n\n${contents}`);
  return contents.replace(/(?<=version = ")\d+\.\d+\.\d+(?=")/g, () => {
    // console.debug("replace version with", version);
    return version;
  });
};
