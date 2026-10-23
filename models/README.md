# Model files

The three retained weights have distinct historical roles. See `MODEL_CARD.md` for hashes, class semantics and limitations.

All `.pt` files are configured for Git LFS. Before the first commit, run:

```bash
git lfs install
git lfs track "*.pt"
```

Do not rename the two soldier weights as if they were interchangeable: one corresponds to the final training record, while the other is the file explicitly referenced by the vehicle backup.
