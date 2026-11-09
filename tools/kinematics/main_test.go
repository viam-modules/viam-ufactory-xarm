package main

import (
	"bytes"
	"io"
	"io/fs"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"go.viam.com/test"
)

// The files under arm/kinematics are what the generator writes from the sources beside them, so
// a change to a source, or to the generator, has to come with regenerated files.
func TestShippedFilesAreFresh(t *testing.T) {
	//nolint:dogsled
	_, file, _, _ := runtime.Caller(0)
	t.Chdir(filepath.Join(filepath.Dir(file), "..", ".."))

	fresh := t.TempDir()
	test.That(t, run(fresh, io.Discard), test.ShouldBeNil)

	shipped := filepath.Join("arm", "kinematics")
	want := listFiles(t, fresh)
	got := listFiles(t, shipped)
	test.That(t, got, test.ShouldResemble, want)
	for _, rel := range want {
		a, err := os.ReadFile(filepath.Join(fresh, rel))
		test.That(t, err, test.ShouldBeNil)
		b, err := os.ReadFile(filepath.Join(shipped, rel))
		test.That(t, err, test.ShouldBeNil)
		if !bytes.Equal(a, b) {
			t.Errorf("%s differs from what the generator writes, run make kinematics", filepath.Join(shipped, rel))
		}
	}
}

func listFiles(t *testing.T, dir string) []string {
	t.Helper()
	var out []string
	err := filepath.WalkDir(dir, func(path string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if d.IsDir() || d.Name() == ".gitattributes" {
			return nil
		}
		rel, err := filepath.Rel(dir, path)
		if err != nil {
			return err
		}
		out = append(out, rel)
		return nil
	})
	test.That(t, err, test.ShouldBeNil)
	return out
}
