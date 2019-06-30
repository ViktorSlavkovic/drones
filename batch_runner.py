#!/usr/bin/python

import subprocess
import os
import sys
import time
import numpy as np

from absl import app
from absl import flags
from absl import logging

FLAGS = flags.FLAGS
flags.DEFINE_string('tcs_dir', '', '...')
flags.DEFINE_string('eval_file', '', '...')
flags.DEFINE_integer('limit_ntcs', 10000, '...')
flags.DEFINE_string('args', '', '...')
flags.DEFINE_integer('best_of', 1, '...')


def main(argv):
    if not os.path.exists(FLAGS.tcs_dir):
        logging.error(f'{FLAGS.tcs_dir} is not a vaild directory.')
        sys.exit(-1)
    info_path = os.path.join(FLAGS.tcs_dir, 'tc_info.txt')
    if not os.path.isfile(info_path):
        logging.error(f'{info_path} is not a valid file.')
        sys.exit(-1)

    ARGS = FLAGS.args.split()

    tc_paths = []
    tc_nd = []
    tc_np = []
    tc_nw = []
    tc_no = []
    tc_m = []
    tc_t = []
    tc_ub = []
    tc_score = []
    tc_quality = []
    tc_exec_time = []

    zs = []

    with open(info_path, 'r') as info:
        num_tc = int(info.readline())
        num_tc = min(num_tc, FLAGS.limit_ntcs)
        logging.info(f'num_tc={num_tc}')
        for tc in range(0, num_tc):
            line = info.readline().split()
            in_file = line[0]
            tc_paths.append(in_file)
            cnd, cnp, cnw, cno, cm, ct, cub = [int(x) for x in line[1:]]
            tc_nd.append(cnd)
            tc_np.append(cnp)
            tc_nw.append(cnw)
            tc_no.append(cno)
            tc_m.append(cm)
            tc_t.append(ct)
            tc_ub.append(cub)
            args = [
                './main',
                f'--problem_file={os.path.join(FLAGS.tcs_dir, in_file)}'
            ] + ARGS
            logging.info(f'running: {args}')

            bcscore = -1
            bct = -1
            for i in range(max(1, FLAGS.best_of)):
                proc = subprocess.Popen(args,
                                        stdout=subprocess.PIPE,
                                        stderr=subprocess.PIPE)
                start = time.time()
                stdout, stderr = proc.communicate()
                ct = time.time() - start
                if proc.returncode != 0:
                    print(stderr)
                    sys.exit(-2)
                cscore = int(stdout.decode('ascii').split()[-1])
                if cscore > bcscore:
                    bcscore = cscore
                    bct = ct

            tc_score.append(bcscore)
            tc_exec_time.append(bct)
            cqual = bcscore * 100.0 / cub if cub != 0 else 100.0
            if (cqual == 0.0):
                zs.append(tc)
            else:
                tc_quality.append(cqual)
            logging.info(f'tc: {tc}/{num_tc} time: {bct} qual: {cqual}')

    print(f'zs: {len(zs)}/{num_tc}')
    print('min, mean, max, var, std')
    print(
        f'Nd: {min(tc_nd)} {np.mean(tc_nd)} {max(tc_nd)} {np.var(tc_nd)} {np.std(tc_nd)}'
    )
    print(
        f'Np: {min(tc_np)} {np.mean(tc_np)} {max(tc_np)} {np.var(tc_np)} {np.std(tc_np)}'
    )
    print(
        f'Nw: {min(tc_nw)} {np.mean(tc_nw)} {max(tc_nw)} {np.var(tc_nw)} {np.std(tc_nw)}'
    )
    print(
        f'No: {min(tc_no)} {np.mean(tc_no)} {max(tc_no)} {np.var(tc_no)} {np.std(tc_no)}'
    )
    print(
        f'M: {min(tc_m)} {np.mean(tc_m)} {max(tc_m)} {np.var(tc_m)} {np.std(tc_m)}'
    )
    print(
        f'T: {min(tc_t)} {np.mean(tc_t)} {max(tc_t)} {np.var(tc_t)} {np.std(tc_t)}'
    )
    print(
        f'et: {min(tc_exec_time)} {np.mean(tc_exec_time)} {max(tc_exec_time)} {np.var(tc_exec_time)} {np.std(tc_exec_time)}'
    )
    print(
        f'q: {min(tc_quality)} {np.mean(tc_quality)} {max(tc_quality)} {np.var(tc_quality)} {np.std(tc_quality)}'
    )

    with open(os.path.join(FLAGS.tcs_dir, FLAGS.eval_file), 'w') as fout:
        fout.write(f'{" ".join(str(x) for x in zs)}\n')
        fout.write(f'{" ".join(str(x) for x in tc_paths)}\n')
        fout.write(f'{" ".join(str(x) for x in tc_nd)}\n')
        fout.write(f'{" ".join(str(x) for x in tc_np)}\n')
        fout.write(f'{" ".join(str(x) for x in tc_nw)}\n')
        fout.write(f'{" ".join(str(x) for x in tc_no)}\n')
        fout.write(f'{" ".join(str(x) for x in tc_m)}\n')
        fout.write(f'{" ".join(str(x) for x in tc_t)}\n')
        fout.write(f'{" ".join(str(x) for x in tc_ub)}\n')
        fout.write(f'{" ".join(str(x) for x in tc_score)}\n')
        fout.write(f'{" ".join(str(x) for x in tc_exec_time)}\n')
        fout.write(f'{" ".join(str(x) for x in tc_quality)}\n')


if __name__ == "__main__":
    app.run(main)
