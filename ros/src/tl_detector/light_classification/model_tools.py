import os
import sys
from functools import partial

CHUNK_SIZE = 1024
MAX_CHUNKS = 40000
DIRECTORY_APPEND_NAME = '_chunks'

def split_file(filename, chunk_size=CHUNK_SIZE, max_chunks=MAX_CHUNKS):
    """Split file into several files and save into another directory.

    Args:
        filename (str): full path to the input file to be splitted
        chunk_size (int): size (bytes) of each chunk
        max_chunks (int): maximum number of chunks

    Returns:
        output (str): full path to the directory where the chunks are stored

    """
    directory = '{}{}'.format(filename, DIRECTORY_APPEND_NAME)
    filename_basename = os.path.basename(filename)

    if not os.path.exists(directory):
        os.mkdir(directory)

    chunknum = 0
    with open(filename, 'rb') as infile:
        for chunk in iter(partial(infile.read, chunk_size * max_chunks), ''):
            ofilename = os.path.join(directory, ('{}_{:04d}'.format(filename_basename, chunknum)))
            outfile = open(ofilename, 'wb')
            outfile.write(chunk)
            outfile.close()
            chunknum += 1


def merge_file(directory, chunk_size=CHUNK_SIZE):
    """Merge chunks into a single file.

    Args:
        directory (str): full path to the directory containing the chunks
        chunk_size (int): size (bytes) of each chunk

    Returns:
        output (str): full path to the output file

    """
    filename = directory.replace(DIRECTORY_APPEND_NAME, '')

    output_dir = os.path.dirname(filename)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    with open(filename, 'wb') as output:
        chunks = os.listdir(directory)
        chunks.sort()
        for fname in chunks:
            fpath = os.path.join(directory, fname)
            with open(fpath, 'rb') as fileobj:
                for chunk in iter(partial(fileobj.read, chunk_size), ''):
                    output.write(chunk)

    return filename
