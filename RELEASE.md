# RELEASE

## Git Steps

1. Bump the version

Edit `Cargo.toml`:

```toml
version = "1.0.1"   # bump from whatever's on crates.io
```

Rules:
- crates.io is append-only — you can never republish the same version. Always bump.
- Use semver: 1.0.0 → 1.0.1 for fixes, 1.0.0 → 1.1.0 for features, 1.0.0 → 2.0.0 for breaking changes.
- Same version is used for both crates.io and PyPI (pyproject.toml has dynamic = ["version"] which pulls from Cargo.toml via maturin).

2. Verify both build configurations

```bash
# Default features (what most users build with)                                                                                                                                                                                                                                                          
cargo build
cargo test --lib
cargo build --bins --examples

# Python feature (what CI builds — this is the one that broke last time)                                                                                                                                                                                                                                 
cargo build --features python 
```

3. Sanity-check what crates.io will receive

```bash
cargo publish --dry-run         # full validation
cargo package --list            # see exactly which files go in the .crate archive
```

4. Configure the `CARGO_REGISTRY_TOKEN` repository secret

The `Publish` workflow authenticates to crates.io using a repository secret named `CARGO_REGISTRY_TOKEN`. If it has not yet been set (or has expired), add it before tagging a release:

**Settings → Secrets and variables → Actions → New repository secret**

- **Name:** `CARGO_REGISTRY_TOKEN` (must match `secrets.CARGO_REGISTRY_TOKEN` in `publish.yml`)
- **Value:** an API token generated at <https://crates.io/settings/tokens> with the `publish-update` scope (and `publish-new` for the first release)

Store the value as a **Secret**, not a **Variable** — variables are exposed in plain text in workflow logs. See [§ Rust](#rust) below for the full token-generation procedure.

5. Commit to GitHub

```bash
git commit -am "Release 1.0.1"
git push
git tag v1.0.1
git push origin v1.0.1
```

---

## GitHub Workflow Actions

## Publish

## PyPi

Once you push the tag, the python-bindings workflow will run, the Publish to PyPI step should fire on the tag ref.

```bash
git tag v1.0.0
git push origin v1.0.0
```

Delete a tag if you need to recreate it:

```bash
git tag -d v1.0.0
git push origin --delete v1.0.0
```

## Rust

Rust crates publish to [crates.io](https://crates.io/), which uses an API token instead of OIDC. The `Publish` workflow (`.github/workflows/publish.yml`) calls `cargo publish` via `katyo/publish-crates@v1` and reads the token from the `CARGO_REGISTRY_TOKEN` repository secret.

### 1. Generate a token on crates.io

1. Sign in at <https://crates.io/> with your GitHub account.
2. Open <https://crates.io/settings/tokens> and click **New Token**.
3. Give it a descriptive name (e.g. `rust_servocom-github-actions`).
4. Scope it down to the minimum needed for CI:
   - `publish-new` — allow publishing brand-new crates
   - `publish-update` — allow publishing new versions
   - Optionally restrict to the `servocom` crate name.
5. Set an expiration (90 days is a good default).
6. Click **Generate Token** and **copy it immediately** — crates.io only shows it once.

### 2. Add the token to GitHub Actions secrets

1. In the repo on GitHub: **Settings → Secrets and variables → Actions**.
2. Click **New repository secret**.
3. Name: `CARGO_REGISTRY_TOKEN` (must match `secrets.CARGO_REGISTRY_TOKEN` in `publish.yml`).
4. Value: paste the token.
5. Click **Add secret**.

### 3. Verify locally (optional)

```bash
cargo login <token>
cargo publish --dry-run
```

`--dry-run` checks that the package would publish cleanly without actually uploading.

### 4. Run the publishing workflow

The `Publish` workflow is `workflow_dispatch`-only — trigger it from the **Actions** tab → **Publish** → **Run workflow**.

### Required `Cargo.toml` metadata

crates.io rejects publishes that are missing any of:

- `name` — must be unique on crates.io. Check availability with:
  ```bash
  curl -sI https://crates.io/api/v1/crates/servocom | head -1
  ```
  `200 OK` means taken; `404` means free.
- `version`
- `description`
- `license` or `license-file`

`repository` is recommended but not strictly required.
