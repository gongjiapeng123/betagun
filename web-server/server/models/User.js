
// mock
const User = {
  user: {
    id: '0',
    username: 'admin',
    password: '666',
  },
  verify: function (username, password) {
    if (this.user.username === username && this.user.password === password) {
      return Promise.resolve(this.user)
    } else {
      return Promise.resolve(null)
    }
  },
  findOne: function (id, cb) {
    if (id === this.user.id) {
      cb(null, this.user)
    } else {
      cb(null, null)
    }
  }
}

export default User